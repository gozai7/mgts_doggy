"""Odometry prediction with robust, observable planar scan-to-map corrections."""

from collections import deque

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from . import bagio, cloud
from .cold_start import moved
from .config import transform
from .trajectory import interpolate


class BodyOdometry:
    def __init__(self):
        self.history = deque(maxlen=180)

    def append(self, msg):
        self.history.append([bagio.ns(msg) * 1e-9, *bagio.pose_values(msg.pose.pose)])

    def at_scan(self, msg, base_from_lidar=None):
        """Deskew cloud_base in the base frame at the first beam using received odometry."""
        if not self.history:
            return None
        history = np.asarray(self.history)
        stamp = bagio.ns(msg) * 1e-9
        if stamp < history[0, 0] - 0.03 or stamp > history[-1, 0] + 0.03:
            return None
        points, fields = cloud.unpack(msg)
        offsets = fields["time"]
        keep = (
            np.isfinite(points).all(1)
            & np.isfinite(offsets)
            & (np.linalg.norm(points, axis=1) > 0.45)
        )
        points, offsets = points[keep], offsets[keep]
        # A float32 beam offset would round Unix seconds to 128-second steps.
        times = np.r_[stamp, stamp + offsets.astype(np.float64)]
        if base_from_lidar is not None:
            points = moved(points, base_from_lidar)
        poses, covered = interpolate(history, times, 0.05)
        first = transform(poses[0])
        if not len(points) or np.mean(covered[1:]) < 0.8:
            return first, np.empty((0, 3))
        body = poses[1:][covered[1:]]
        odom_points = Rotation.from_quat(body[:, 3:]).apply(points[covered[1:]]) + body[:, :3]
        return first, moved(odom_points, np.linalg.inv(first))


class MapTracker:
    def __init__(self, model, map_from_odom):
        self.tree = cKDTree(model)
        _, neighbors = self.tree.query(model, k=10)
        offsets = model[neighbors] - model[neighbors].mean(axis=1, keepdims=True)
        eig, axes = np.linalg.eigh(offsets.transpose(0, 2, 1) @ offsets)
        self.normals = axes[:, :, 0]
        self.flat = (eig[:, 1] > 2.5 * eig[:, 0]) & (eig[:, 1] > 1e-5)
        self.map_from_odom = map_from_odom

    def update(self, odom, points):
        prior = self.map_from_odom @ odom
        # Alternate beams are reserved to check each correction against unseen returns.
        if len(points) < 60:
            return prior, "odometry"
        source = moved(points[::2], prior)
        dist, ids = self.tree.query(source)
        keep = (dist < 0.3) & self.flat[ids]
        if keep.sum() < 30:
            return prior, "odometry"
        normals = self.normals[ids[keep]]
        p = source[keep]
        diff = np.einsum("ij,ij->i", normals, self.tree.data[ids[keep]] - p)
        arm = p - prior[:3, 3]
        J = np.column_stack((normals[:, :2], arm[:, 0] * normals[:, 1] - arm[:, 1] * normals[:, 0]))
        weights = np.minimum(1.0, 0.045 / np.maximum(abs(diff), 1e-6))
        # Use only axes constrained by the local surface geometry.
        H = (J.T * weights) @ J / len(J)
        eig, axes = np.linalg.eigh(H)
        gain = np.where(eig > 0.025, 1 / (eig + 0.06), 0.0)
        step = axes @ (gain * (axes.T @ (J.T @ (weights * diff) / len(J))))
        if not np.any(gain) or np.linalg.norm(step[:2]) > 0.25 or abs(step[2]) > 0.12:
            return prior, "odometry"
        candidate = prior.copy()
        candidate[:2, 3] += 0.4 * step[:2]
        candidate[:3, :3] = Rotation.from_euler("z", 0.4 * step[2]).as_matrix() @ prior[:3, :3]
        held = points[1::2]

        def score(pose):
            return np.minimum(self.tree.query(moved(held, pose))[0], 0.30).mean()

        if score(candidate) >= score(prior):
            return prior, "odometry"
        self.map_from_odom = candidate @ np.linalg.inv(odom)
        return candidate, "lidar"
