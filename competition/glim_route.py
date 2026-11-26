"""Express the offline GLIM B route in A's frame from one automatic localization."""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import pose7, transform, write_json
from .trajectory import interpolate, read_tum, save


def align(glim, odom, attempts):
    lock = next((attempt for attempt in attempts if attempt["separated"]), None)
    if lock is None:
        raise ValueError("GLIM B alignment requires a confirmed automatic localization")

    # The search reports A <- body at the FIRST B scan. GLIM starts later, after
    # IMU initialization. Propagate over only this gap with relative body odometry.
    first_stamp = lock["stamp_ns"] * 1e-9
    at_glim, valid = interpolate(odom, glim[:, 0], max_gap=0.1)
    material = np.flatnonzero(valid & (glim[:, 0] >= first_stamp))
    if not material.size:
        raise ValueError("No GLIM B / body odometry overlap after the first B scan")
    index = material[0]
    a_from_odom = transform(lock["initial_pose"]) @ np.linalg.inv(
        transform(lock["first_odometry_pose"])
    )
    a_from_glim = (
        a_from_odom @ transform(at_glim[index]) @ np.linalg.inv(transform(glim[index, 1:]))
    )
    route = glim.copy()
    route[:, 1:4] = glim[:, 1:4] @ a_from_glim[:3, :3].T + a_from_glim[:3, 3]
    route[:, 4:] = (
        Rotation.from_matrix(a_from_glim[:3, :3]) * Rotation.from_quat(glim[:, 4:])
    ).as_quat()
    return route, {
        "first_scan_stamp_ns": lock["stamp_ns"],
        "initialization_prefix_s": lock["prefix_s"],
        "anchor_stamp_s": float(glim[index, 0]),
        "T_A_glim_B_xyzw": pose7(a_from_glim),
        "poses": len(route),
    }


def export(dest, glim_b_mapping):
    dest = Path(dest)
    glim = read_tum(Path(glim_b_mapping) / "trajectory_optimized.tum")
    attempts = json.loads((dest / "initialization.json").read_text())
    odom = read_tum(Path(glim_b_mapping) / "kinematic.tum")
    route, anchor = align(glim, odom, attempts)
    save(dest / "glim_b_in_map.tum", route, csv=False)
    write_json(dest / "glim_b_alignment.json", anchor)
