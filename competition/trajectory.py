"""TUM uses seconds, metres and qx qy qz qw. No implicit frame alignment."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .config import pose7, transform


def read_tum(path):
    a = np.loadtxt(path, comments="#", ndmin=2)
    if a.ndim != 2 or a.shape[1] != 8 or not len(a) or not np.isfinite(a).all():
        raise ValueError(f"Invalid/empty TUM: {path}")
    if np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError(f"TUM timestamps are not strictly increasing: {path}")
    norm = np.linalg.norm(a[:, 4:], axis=1)
    if np.any(abs(norm - 1) > .01):
        raise ValueError("Invalid trajectory quaternion")
    a[:, 4:] /= norm[:, None]
    return a


def save(path, a, csv=True):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, a, fmt="%.9f")
    if csv:
        np.savetxt(path.with_suffix(".csv"), a, delimiter=",", fmt="%.9f",
                   header="t,x,y,z,qx,qy,qz,qw", comments="")


def change_body(a, T_old_new):
    b = a.copy()
    for i, row in enumerate(a):
        b[i, 1:] = pose7(transform(row[1:]) @ T_old_new)
    return b


def interpolate(a, times, max_gap=.5):
    """Returns poses and validity; never extrapolate or bridge long dropouts."""
    times = np.asarray(times, dtype=float)
    j = np.searchsorted(a[:, 0], times, side="left")
    k = np.clip(j, 0, len(a) - 1)
    exact = abs(a[k, 0] - times) <= 1e-7
    lo, hi = np.clip(j - 1, 0, len(a) - 1), k
    valid = exact | ((j > 0) & (j < len(a)) & (a[hi, 0] - a[lo, 0] <= max_gap))
    clipped = np.clip(times, a[0, 0], a[-1, 0])
    out = np.empty((len(times), 7))
    for d in range(3):
        out[:, d] = np.interp(clipped, a[:, 0], a[:, d + 1])
    if len(a) == 1:
        out[:, 3:] = a[0, 4:]
    else:
        out[:, 3:] = Slerp(a[:, 0], Rotation.from_quat(a[:, 4:]))(clipped).as_quat()
    return out, valid


def metrics(a, expected_span=None, gap_limit=.5):
    dt = np.diff(a[:, 0]); dx = np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1)
    dr = (Rotation.from_quat(a[:-1, 4:]).inv() * Rotation.from_quat(a[1:, 4:])).magnitude()
    covered = float(dt[dt <= gap_limit].sum())
    duration = float(a[-1, 0] - a[0, 0])
    denominator = expected_span[1] - expected_span[0] if expected_span is not None else duration
    return {"poses": len(a), "duration_s": duration, "path_length_m": float(dx.sum()),
            "maximum_gap_s": float(dt.max()) if len(dt) else None,
            "maximum_step_m": float(dx.max()) if len(dx) else None,
            "maximum_rotation_step_rad": float(dr.max()) if len(dr) else None,
            "covered_time_s": covered, "coverage_fraction": min(1.0, covered / denominator) if denominator > 0 else None,
            "coverage_denominator": "input sensor span" if expected_span is not None else "trajectory span only",
            "absolute_accuracy": None, "endpoint_distance_is_not_loop_error": float(np.linalg.norm(a[-1, 1:4] - a[0, 1:4]))}
