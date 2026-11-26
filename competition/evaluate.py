from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .trajectory import read_tum, interpolate, metrics
from .config import write_json


def compare(a_path, b_path, T_a_b=None, max_gap=.5):
    """Pose disagreement requires an explicit transform, even identity."""
    if T_a_b is None:
        raise ValueError("Provide T_a_b; map origins from separate SLAM runs are not automatically aligned")
    from .config import transform
    T = transform(T_a_b)
    a, b = read_tum(a_path), read_tum(b_path)
    poses, valid = interpolate(b, a[:, 0], max_gap)
    if not valid.any():
        raise ValueError("No valid timestamp overlap")
    xyz_b = poses[valid, :3] @ T[:3, :3].T + T[:3, 3]
    d = np.linalg.norm(xyz_b - a[valid, 1:4], axis=1)
    return {"matched": int(valid.sum()), "position_disagreement_rmse_m": float(np.sqrt(np.mean(d*d))),
            "p95_m": float(np.percentile(d, 95)), "interpretation": "disagreement, not absolute accuracy",
            "T_a_b": T_a_b}


def ground_truth(reference_path, estimated_path, T_reference_map, delta_s=1., max_gap=.5):
    """APE and fixed-time RPE, with explicit rigid alignment and no scale fitting.

    Both inputs must describe the same physical body frame in metres and seconds.
    RPE pairs are overlapping; reference and estimate are interpolated at t+delta.
    """
    from .config import transform
    if delta_s <= 0 or max_gap <= 0:
        raise ValueError("delta_s and max_gap must be positive")
    T = transform(T_reference_map)
    ref, est = read_tum(reference_path), read_tum(estimated_path)
    est[:, 1:4] = est[:, 1:4] @ T[:3, :3].T + T[:3, 3]
    est[:, 4:] = (Rotation.from_matrix(T[:3, :3]) * Rotation.from_quat(est[:, 4:])).as_quat()
    r0, ok0 = interpolate(ref, est[:, 0], max_gap)
    if not ok0.any():
        raise ValueError("No matched ground truth poses")
    xyz_error = np.linalg.norm(est[ok0, 1:4] - r0[ok0, :3], axis=1)
    angle_error = (Rotation.from_quat(r0[ok0, 3:]).inv() * Rotation.from_quat(est[ok0, 4:])).magnitude()
    def statistics(a):
        return {"rmse": float(np.sqrt(np.mean(a*a))), "median": float(np.median(a)),
                "p95": float(np.percentile(a, 95)), "max": float(a.max())} if len(a) else None
    future = est[:, 0] + delta_s
    r1, okr = interpolate(ref, future, max_gap)
    e1, oke = interpolate(est, future, max_gap)
    trans_errors, rot_errors = [], []
    for i in np.flatnonzero(ok0 & okr & oke):
        dr = np.linalg.inv(transform(r0[i])) @ transform(r1[i])
        de = np.linalg.inv(transform(est[i, 1:])) @ transform(e1[i])
        error = np.linalg.inv(dr) @ de
        trans_errors.append(np.linalg.norm(error[:3, 3]))
        rot_errors.append(Rotation.from_matrix(error[:3, :3]).magnitude())
    return {"reference": str(reference_path), "estimated": str(estimated_path),
            "alignment": "explicit rigid transform; no fitted scale", "T_reference_map": T_reference_map,
            "matched_ape": int(ok0.sum()), "ape_translation_m": statistics(xyz_error),
            "ape_rotation_rad": statistics(angle_error), "rpe_delta_s": delta_s,
            "rpe_pairs": len(trans_errors), "rpe_translation_m": statistics(np.array(trans_errors)),
            "rpe_rotation_rad": statistics(np.array(rot_errors)), "max_interpolation_gap_s": max_gap}


def evaluate(root):
    root = Path(root)
    result = {"absolute_accuracy": None, "wall_thickness_m": None, "icp_fitness": None,
              "inlier_rmse_m": None, "confirmed_loop_error_m": None,
              "note": "Unavailable metrics are null. No ground truth was assumed."}
    for phase, filename in (("mapping", "trajectory_optimized.tum"), ("localization", "trajectory.tum")):
        path = root / phase / filename
        if path.exists():
            span = None
            manifest = root / "normalized" / phase / "manifest.json"
            if manifest.exists():
                span = [v * 1e-9 for v in json.loads(manifest.read_text())["spans_ns"]["lidar"]]
            result[phase] = metrics(read_tum(path), span)
    diag = root / "localization" / "diagnostics.json"
    if diag.exists():
        result["localization_diagnostics"] = json.loads(diag.read_text())
    write_json(root / "diagnostics" / "metrics.json", result)
    (root / "report.md").write_text("# Диагностический отчёт\n\nБез ground truth эти показатели не являются абсолютной точностью.\n\n```json\n" + json.dumps(result, ensure_ascii=False, indent=2) + "\n```\n")
    return result
