"""PointCloud2 layouts including row padding, big-endian fields and point time."""
import numpy as np
from .bagio import STORE, header, ns

DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
TIME_FIELDS = ("time", "t", "timestamp", "time_stamp", "offset_time")
SCALE = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def fields(msg):
    names, formats, offsets = [], [], []
    endian = ">" if msg.is_bigendian else "<"
    for f in msg.fields:
        if f.name in names or f.datatype not in DTYPES or f.count < 1:
            raise ValueError(f"Invalid/duplicate PointField {f.name}")
        typ = np.dtype(endian + DTYPES[f.datatype])
        if f.offset < 0 or f.offset + typ.itemsize * f.count > msg.point_step:
            raise ValueError(f"PointField {f.name} exceeds point_step")
        names.append(f.name); offsets.append(f.offset)
        formats.append(typ if f.count == 1 else (typ, (f.count,)))
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError("Truncated or invalid PointCloud2 layout")
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def unpack(msg):
    a = fields(msg)
    if not all(x in a.dtype.names and a[x].ndim == 1 for x in ("x", "y", "z")):
        raise ValueError("PointCloud2 requires scalar x/y/z")
    return np.column_stack([a[x] for x in ("x", "y", "z")]).astype(float), a


def point_seconds(a, stamp_ns, config, period):
    if config["mode"] == "none":
        return None
    name = config["field"]
    if name not in a.dtype.names:
        raise ValueError(f"Point time field {name!r} missing; explicitly select mode:none to disable deskew")
    # Keep integer epoch nanoseconds precise until after the large common epoch is removed.
    t = np.asarray(a[name], dtype=np.longdouble) * np.longdouble(str(SCALE[config["unit"]]))
    if config["mode"] == "absolute":
        t -= np.longdouble(stamp_ns) / 1_000_000_000
    t += float(config.get("offset_s", 0))
    if t.ndim != 1 or not np.isfinite(t).all() or (len(t) and np.max(abs(t)) > period * 2):
        raise ValueError("Invalid point times: check units, absolute/relative convention and scan stamp")
    return np.asarray(t, dtype=float)


def pack(xyz, n, frame, intensity=None, times=None):
    desc = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")]
    if times is not None:
        desc.append(("time", "<f8"))
    a = np.zeros(len(xyz), dtype=desc)
    for i, axis in enumerate(("x", "y", "z")):
        a[axis] = xyz[:, i]
    if intensity is not None:
        a["intensity"] = intensity
    if times is not None:
        a["time"] = times
    fs = [STORE.types["sensor_msgs/msg/PointField"](name=name, offset=a.dtype.fields[name][1],
          datatype=8 if name == "time" else 7, count=1) for name in a.dtype.names]
    return STORE.types["sensor_msgs/msg/PointCloud2"](
        header=header(n, frame), height=1, width=len(a), fields=fs, is_bigendian=False,
        point_step=a.dtype.itemsize, row_step=a.nbytes, data=np.frombuffer(a.tobytes(), dtype=np.uint8).copy(),
        is_dense=bool(np.isfinite(xyz).all()))


def normalize(msg, c):
    xyz, a = unpack(msg)
    xyz *= {"m": 1.0, "mm": .001, "cm": .01}[c["units"]["xyz"]]
    times = point_seconds(a, ns(msg), c["point_time"], c["lidar"]["scan_period_s"])
    r = np.linalg.norm(xyz, axis=1)
    lo, hi = c["lidar"]["range_m"]
    mask = np.isfinite(xyz).all(axis=1) & (r >= lo) & (r <= hi)
    # A measured body box is in base coordinates, including the sensor lever arm.
    box = c["lidar"].get("body_box_base_m")
    if box is not None:
        from .config import transform
        T = transform(c["extrinsics"]["T_base_lidar"])
        base = xyz @ T[:3, :3].T + T[:3, 3]
        mask &= ~np.all((base >= box[0]) & (base <= box[1]), axis=1)
    ids = np.flatnonzero(mask)
    if times is not None:
        ids = ids[np.argsort(times[ids], kind="stable")]
    if len(ids) < 5:
        raise ValueError("Fewer than five valid cloud points after filtering")
    intensity_name = c["lidar"].get("intensity_field", "intensity")
    intensity = np.asarray(a[intensity_name], dtype=float)[ids] if intensity_name in a.dtype.names else None
    n = ns(msg) + round(c["time_offsets_s"]["lidar"] * 1e9)
    return pack(xyz[ids], n, c["frames"]["lidar"], intensity, None if times is None else times[ids])
