"""Causal SE(3) registration of recent B returns against the fixed A surface map."""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from .cold_start import moved
from .config import pose7
from .go2_full_route import registered_prefix


class WindowTracker:
    def __init__(self, tracker):
        self.target = tracker
        self.window = deque()
        self.last_fit = None

    @staticmethod
    def prepare(odom, points):
        return moved(points[::3], odom)

    def update(self, stamp, odom, points, *, prefix=False):
        prior = self.target.map_from_odom @ odom
        if len(points):
            self.window.append((stamp, points))
        while self.window and stamp - self.window[0][0] > 1_600_000_000:
            self.window.popleft()
        interval = 900_000_000 if prefix else 190_000_000
        if len(self.window) < 2 or (
            self.last_fit is not None and stamp - self.last_fit < interval
        ):
            return prior, "odometry"
        self.last_fit = stamp
        raw = np.vstack([part for _, part in self.window])
        _, ids = np.unique(
            np.floor(raw / 0.11).astype(np.int32), axis=0, return_index=True
        )
        sample = raw[ids[:: max(1, int(np.ceil(len(ids) / 1300)))]]
        if len(sample) < 90:
            return prior, "odometry"
        pose = self.target.map_from_odom.copy()
        for _ in range(5):
            xyz = moved(sample, pose)
            distances, indices = self.target.tree.query(xyz)
            good = (distances < 0.65) & self.target.flat[indices]
            if good.sum() < 65:
                break
            indices = indices[good]
            normals = self.target.normals[indices]
            residual = np.einsum(
                "ij,ij->i", self.target.tree.data[indices] - xyz[good], normals
            )
            center = pose[:3, :3] @ odom[:3, 3] + pose[:3, 3]
            jac = np.column_stack((normals, np.cross(xyz[good] - center, normals)))
            # Balance dominant surface directions so the floor cannot overwhelm the walls.
            direction = abs(normals).argmax(axis=1)
            mass = np.maximum(np.bincount(direction, minlength=3), 0.12 * len(normals))
            weights = (
                np.minimum(1.0, 0.08 / np.maximum(abs(residual), 1e-10))
                / mass[direction]
            )
            hessian = (jac.T * weights) @ jac / weights.sum()
            gradient = jac.T @ (weights * residual) / weights.sum()
            eig, axes = np.linalg.eigh(hessian)
            # A corridor may constrain lateral motion only. Keep the odometry prediction on weak axes.
            gain = np.where(eig > 0.009, 1 / (eig + 0.035), 0.0)
            step = axes @ (gain * (axes.T @ gradient))
            translation, angle = np.linalg.norm(step[:3]), np.linalg.norm(step[3:])
            if translation < 0.002 and angle < 0.001:
                break
            step *= min(
                0.8, 0.13 / max(0.0001, translation), 0.036 / max(0.0001, angle)
            )
            rotation = Rotation.from_rotvec(step[3:]).as_matrix()
            pose[:3, 3] = rotation @ (pose[:3, 3] - center) + center + step[:3]
            pose[:3, :3] = rotation @ pose[:3, :3]
        estimated = pose @ odom
        rotation = Rotation.from_matrix(estimated[:3, :3] @ prior[:3, :3].T).as_rotvec()
        delta = estimated[:3, 3] - prior[:3, 3]
        if np.linalg.norm(delta) < 1e-9 and np.linalg.norm(rotation) < 1e-9:
            return prior, "odometry"
        scale = min(
            0.55,
            0.17 / max(np.linalg.norm(delta), 1e-6),
            0.04 / max(np.linalg.norm(rotation), 1e-6),
        )
        prior[:3, 3] += scale * delta
        prior[:3, :3] = (
            Rotation.from_rotvec(scale * rotation).as_matrix() @ prior[:3, :3]
        )
        self.target.map_from_odom = prior @ np.linalg.inv(odom)
        return prior, "lidar"

    def replay_prefix(self, pending, lock, first_scan_ns, first_odom_ns):
        rows = []
        if not pending or pending[0][0] != first_scan_ns:
            rows = registered_prefix(
                [], lock, self.target.map_from_odom, first_scan_ns, first_odom_ns
            )
        lidar_stamps = set()
        for stamp, body, points in pending:
            pose, source = self.update(stamp, body, points, prefix=True)
            rows.append((stamp, pose7(pose)))
            if source == "lidar":
                lidar_stamps.add(stamp)
        return rows, lidar_stamps
