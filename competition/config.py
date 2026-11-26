"""Explicit sensor contracts; no guessed units or robot calibrations."""
from pathlib import Path
import copy
import hashlib
import json
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def write_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def transform(values):
    a = np.asarray(values, dtype=float)
    if a.shape != (7,) or not np.isfinite(a).all():
        raise ValueError("Extrinsic must be [x,y,z,qx,qy,qz,qw], all finite")
    if not np.isclose(np.linalg.norm(a[3:]), 1, atol=1e-3):
        raise ValueError("Extrinsic quaternion must have unit norm")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(a[3:]).as_matrix()
    t[:3, 3] = a[:3]
    return t


def pose7(t):
    return np.r_[t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_quat()].tolist()


def load(path):
    c = yaml.safe_load(Path(path).read_text())
    if not isinstance(c, dict) or c.get("schema_version") != 1:
        raise ValueError("Expected dataset schema_version: 1")
    return c


def validate(c, require_imu=True):
    for k in ("base", "lidar", "imu", "odom", "map"):
        if not isinstance(c["frames"].get(k), str) or not c["frames"][k]:
            raise ValueError(f"Set frames.{k} explicitly")
    if len(set(c["frames"].values())) != len(c["frames"]):
        raise ValueError("Use distinct canonical frame names (measured identity extrinsics are allowed)")
    for k in ("T_base_lidar", "T_base_imu") if require_imu else ("T_base_lidar",):
        if c["extrinsics"].get(k) is None:
            raise ValueError(f"Missing measured extrinsics.{k}; identity is NOT assumed")
        transform(c["extrinsics"][k])
    if c["units"].get("xyz") not in ("m", "mm", "cm"):
        raise ValueError("Set units.xyz to m/cm/mm")
    if require_imu and (c["units"].get("acceleration") not in ("m/s2", "g") or
                        c["units"].get("angular_velocity") not in ("rad/s", "deg/s")):
        raise ValueError("Set IMU units: acceleration m/s2|g; angular_velocity rad/s|deg/s")
    pt = c["point_time"]
    if pt.get("mode") not in ("relative", "absolute", "none"):
        raise ValueError("point_time.mode must be relative, absolute or none")
    if pt["mode"] != "none" and (not pt.get("field") or pt.get("unit") not in ("s", "ms", "us", "ns")):
        raise ValueError("Set exact point_time.field and point_time.unit")
    if not 0 < float(c["lidar"]["scan_period_s"]) < 10:
        raise ValueError("Invalid lidar.scan_period_s")
    lo, hi = c["lidar"]["range_m"]
    if not 0 <= lo < hi:
        raise ValueError("Invalid lidar.range_m")
    for v in c["time_offsets_s"].values():
        if not np.isfinite(v):
            raise ValueError("Time offsets must be finite")
    if c["localization"]["prediction"] not in ("constant_velocity", "kinematic", "imu", "external_tf"):
        raise ValueError("Choose one localization predictor")
    occ = c["occupancy"]
    for key in ("resolution_m", "angular_bins", "every_n_scans", "min_hit_scans", "max_pose_gap_s", "max_cells"):
        if not np.isfinite(occ[key]) or occ[key] <= 0:
            raise ValueError(f"occupancy.{key} must be positive")
    for key in ("angular_bins", "every_n_scans", "min_hit_scans", "max_cells"):
        if not isinstance(occ[key], int):
            raise ValueError(f"occupancy.{key} must be an integer")
    if not np.isfinite(occ["floor_z_m"]) or not occ["height_m"][0] < occ["height_m"][1]:
        raise ValueError("Invalid floor/height settings")
    return c


def resolve(c, reader, require_imu=True):
    c = copy.deepcopy(c)
    kinds = {"lidar": "sensor_msgs/msg/PointCloud2", "imu": "sensor_msgs/msg/Imu",
             "odometry": "nav_msgs/msg/Odometry"}
    for sensor, msgtype in kinds.items():
        topic = c["topics"].get(sensor)
        matches = sorted({x.topic for x in reader.connections if x.msgtype == msgtype})
        if topic == "auto":
            if len(matches) != 1:
                raise ValueError(f"Ambiguous/missing {sensor} topics: {matches}; set topics.{sensor}")
            c["topics"][sensor] = matches[0]
        elif topic is not None and topic not in matches:
            raise ValueError(f"{sensor} topic {topic!r} is not {msgtype}; candidates={matches}")
        if (sensor == "lidar" or sensor == "imu" and require_imu) and not c["topics"][sensor]:
            raise ValueError(f"{sensor} is required")
    return validate(c, require_imu)


def fingerprint(c):
    return hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()


def sensor_contract(c):
    return {k: c[k] for k in ("schema_version", "topics", "frames", "source_frames", "extrinsics",
                             "units", "point_time", "time_offsets_s", "lidar")}


def phase_fingerprint(c, phase):
    effective = phase_config(c, "mapping" if phase == "baseline" else phase)
    contract = sensor_contract(effective)
    if phase == "baseline":
        contract["topics"] = dict(contract["topics"], imu=None, odometry=None)
    else:
        contract[phase] = effective[phase]
    contract["bag"] = c["bags"]["mapping" if phase == "baseline" else phase]
    return fingerprint(contract)


def phase_config(c, phase):
    """Allow different topics/calibrations in the two bags without editing code."""
    result = copy.deepcopy(c)
    def merge(target, source):
        for key, value in source.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = copy.deepcopy(value)
    merge(result, c.get("phase_overrides", {}).get(phase, {}))
    return result


def source_identity(path):
    """Cheap cache identity, not a claim of a cryptographic hash of all sensor bytes."""
    root = Path(path).resolve()
    files = [root] if root.is_file() else sorted(p for p in root.rglob("*") if p.is_file())
    if not files:
        raise ValueError(f"Empty bag: {path}")
    return [{"path": p.name if root.is_file() else str(p.relative_to(root)),
             "bytes": p.stat().st_size, "mtime_ns": p.stat().st_mtime_ns} for p in files]
