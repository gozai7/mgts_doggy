"""Estimate additive IMU header correction using independent angular motion."""
import numpy as np
from scipy.spatial.transform import Rotation
from . import bagio
from .config import transform
from .trajectory import read_tum


def correlate(imu_times, imu_omega, ref_times, ref_omega, max_shift=.1, step=.001):
    if max_shift <= 0 or step <= 0:
        raise ValueError("Shift range and step must be positive")
    it, iw, rt, rw = map(np.asarray, (imu_times, imu_omega, ref_times, ref_omega))
    if len(it) < 20 or len(rt) < 20 or np.any(np.diff(it) <= 0) or np.any(np.diff(rt) <= 0):
        raise ValueError("Need at least 20 strictly increasing samples from both sensors")
    lo, hi = max(it[0] + max_shift, rt[0]), min(it[-1] - max_shift, rt[-1])
    ts = rt[(rt >= lo) & (rt <= hi)]
    ref = rw[(rt >= lo) & (rt <= hi)]
    if len(ts) < 20 or np.sqrt(np.mean(np.var(ref, axis=0))) < .02:
        return {"status": "insufficient_excitation", "offset_s": None, "applied": False}
    ref = ref - ref.mean(axis=0)
    offsets = np.arange(-max_shift, max_shift + step / 2, step)
    scores = []
    for offset in offsets:
        # corrected_imu_time = recorded_imu_time + offset
        v = np.column_stack([np.interp(ts - offset, it, iw[:, k]) for k in range(3)])
        v -= v.mean(axis=0)
        den = np.linalg.norm(v) * np.linalg.norm(ref)
        scores.append(float(np.sum(v * ref) / den) if den > 1e-12 else 0.)
    j = int(np.argmax(scores))
    valid = j not in (0, len(offsets) - 1) and scores[j] >= .5
    return {"status": "candidate_review_required" if valid else "unreliable", "offset_s": float(offsets[j]) if valid else None,
            "peak_correlation": scores[j], "offset_grid_s": offsets.tolist(), "scores": scores,
            "applied": False, "sign": "corrected IMU stamp = input IMU stamp + offset_s"}


def estimate(path, independent_tum, c, max_shift=.1):
    ts, omega = [], []
    R = transform(c["extrinsics"]["T_base_imu"])[:3, :3]
    with bagio.reader(path) as r:
        for conn, _, raw in r.messages(connections=[x for x in r.connections if x.topic == "/competition/imu"]):
            m = r.deserialize(raw, conn.msgtype)
            ts.append(bagio.ns(m) * 1e-9); omega.append(R @ bagio.vec(m.angular_velocity))
    a = read_tum(independent_tum)  # base poses, not an IMU-fused reference
    rotations = Rotation.from_quat(a[:, 4:])
    rw = (rotations[:-1].inv() * rotations[1:]).as_rotvec() / np.diff(a[:, 0])[:, None]
    return correlate(ts, omega, (a[:-1, 0] + a[1:, 0]) / 2, rw, max_shift)
