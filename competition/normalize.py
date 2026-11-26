"""Build an immutable canonical bag. Sorting uses disk, not a full bag in RAM."""
from pathlib import Path
import sqlite3
import shutil
import copy
import numpy as np
import yaml
from rosbags.rosbag2 import Writer
from . import bagio, cloud
from .config import resolve, write_json, fingerprint, source_identity, sensor_contract


def normalize_bag(path, output, c, require_imu=True):
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(output.name + ".partial")
    staging.mkdir()
    db = sqlite3.connect(staging / "sort.sqlite")
    db.execute("CREATE TABLE messages(t INTEGER, seq INTEGER PRIMARY KEY, topic TEXT, type TEXT, data BLOB)")
    counts, previous, spans, source_frames = {}, {}, {}, {}
    seq = 0

    def insert(topic, n, msg):
        nonlocal seq
        if n < 0:
            raise ValueError("Negative normalized timestamp")
        typ = msg.__msgtype__
        db.execute("INSERT INTO messages VALUES (?,?,?,?,?)", (int(n), seq, topic, typ,
                   bytes(bagio.STORE.serialize_cdr(msg, typ))))
        seq += 1

    try:
        with bagio.reader(path) as r:
            c = resolve(c, r, require_imu)
            selected = {v: k for k, v in c["topics"].items() if v and k in ("lidar", "imu", "odometry")}
            for conn, _, data in r.messages(connections=[x for x in r.connections if x.topic in selected]):
                sensor = selected[conn.topic]
                m = r.deserialize(data, conn.msgtype)
                stamp = bagio.ns(m)
                if stamp <= 0 or stamp <= previous.get(sensor, -1):
                    raise ValueError(f"{sensor}: zero, duplicate or decreasing header stamp {stamp}")
                previous[sensor] = stamp
                expected = c.get("source_frames", {}).get(sensor)
                seen = m.header.frame_id
                if not seen or expected is not None and seen != expected:
                    raise ValueError(f"Unexpected {sensor} frame {seen!r}; configured {expected!r}")
                if sensor in source_frames and source_frames[sensor] != seen:
                    raise ValueError(f"Changing {sensor} frame_id")
                source_frames[sensor] = seen
                if sensor == "lidar":
                    m = cloud.normalize(m, c)
                else:
                    n = stamp + round(c["time_offsets_s"][sensor] * 1e9)
                    bagio.set_stamp(m, n)
                    if sensor == "imu":
                        m.header.frame_id = c["frames"]["imu"]
                        for key, scale in (("linear_acceleration", 9.80665 if c["units"]["acceleration"] == "g" else 1),
                                           ("angular_velocity", np.pi / 180 if c["units"]["angular_velocity"] == "deg/s" else 1)):
                            values = bagio.vec(getattr(m, key)) * scale
                            if not np.isfinite(values).all():
                                raise ValueError("Nonfinite IMU sample")
                            bagio.assign_vec(getattr(m, key), values)
                            cov = getattr(m, key + "_covariance")
                            if cov[0] != -1:
                                setattr(m, key + "_covariance", np.asarray(cov) * scale * scale)
                    else:
                        child = c.get("source_frames", {}).get("odometry_child")
                        if child is None or m.child_frame_id != child:
                            raise ValueError("Set source_frames.odometry_child to the physical base frame of input odometry")
                        from .config import transform
                        transform(bagio.pose_values(m.pose.pose))
                        # Odometry must be SI. Do not convert rotations/covariances by guessing.
                        m.header.frame_id = c["frames"]["odom"]
                        m.child_frame_id = c["frames"]["base"]
                n = bagio.ns(m)
                insert("/competition/" + sensor, n, m)
                counts[sensor] = counts.get(sensor, 0) + 1
                spans.setdefault(sensor, [n, n])[1] = n
            if not counts.get("lidar") or require_imu and not counts.get("imu"):
                raise ValueError("Required sensor contains no messages")
            if require_imu and max(spans["lidar"][0], spans["imu"][0]) >= min(spans["lidar"][1], spans["imu"][1]):
                raise ValueError("LiDAR and IMU have no temporal overlap after configured offsets")
        n0 = min(v[0] for v in spans.values())
        tfs = [bagio.tf_message(n0, c["frames"]["base"], c["frames"][k], c["extrinsics"]["T_base_" + k])
               for k in ("lidar", "imu") if c["extrinsics"].get("T_base_" + k) is not None]
        insert("/tf_static", n0, bagio.STORE.types["tf2_msgs/msg/TFMessage"](transforms=tfs))
        db.commit()
        with Writer(staging / "bag", version=9) as w:
            conns = {}
            for n, topic, typ, data in db.execute("SELECT t,topic,type,data FROM messages ORDER BY t,seq"):
                if topic not in conns:
                    conns[topic] = w.add_connection(topic, typ, typestore=bagio.STORE)
                w.write(conns[topic], n, data)
        db.close()
        (staging / "sort.sqlite").unlink()
        (staging / "config_resolved.yaml").write_text(yaml.safe_dump(c, allow_unicode=True))
        report = {"complete": True, "source": str(path), "source_identity": source_identity(path), "config_sha256": fingerprint(c),
                  "sensor_contract_sha256": fingerprint(sensor_contract(c)),
                  "counts": counts, "spans_ns": spans, "source_frames": source_frames,
                  "deskew": "upstream" if c["point_time"]["mode"] != "none" else "disabled_explicitly",
                  "bag_time_policy": "corrected header time; sorted on disk", "original_modified": False}
        write_json(staging / "manifest.json", report)
        staging.rename(output)
        return report
    except BaseException:
        db.close()
        # Preserve partial data for diagnosis; never masquerade as a completed bag.
        raise
