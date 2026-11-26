"""Match a short B submap to a saved A map using only relative B odometry."""

import argparse
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from competition import bagio, cloud, pcd
from competition.cold_start import distinct, distance, fit, moved, sample
from competition.config import pose7, transform, write_json
from competition.trajectory import read_tum


def prefix(bag, seconds):
    poses, scans = [], []
    with bagio.reader(bag) as reader:
        topics = [
            c
            for c in reader.connections
            if c.topic in ("/utlidar/cloud_base", "/utlidar/robot_odom")
        ]
        for conn, _, raw in reader.messages(connections=topics):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = bagio.ns(msg)
            if conn.topic.endswith("odom"):
                poses.append((stamp, transform(bagio.pose_values(msg.pose.pose))))
            else:
                scans.append((stamp, cloud.unpack(msg)[0]))
                if stamp - scans[0][0] >= seconds * 1e9:
                    break
    timestamps = np.array([t for t, _ in poses])
    anchor = None
    sets = [[], []]
    for i, (stamp, xyz) in enumerate(scans):
        j = max(0, np.searchsorted(timestamps, stamp, side="right") - 1)
        if anchor is None:
            anchor = np.linalg.inv(poses[j][1])
        pts = xyz[np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.45)]
        sets[i % 2].append(moved(pts, anchor @ poses[j][1]))
    return (
        [sample(np.vstack(s), 0.10, 3200) for s in sets],
        scans[0][0],
        (scans[-1][0] - scans[0][0]) * 1e-9,
        pose7(np.linalg.inv(anchor)),
    )


def candidates(model, query, ref, step=0.20):
    # Work in height bands to avoid voting for floor returns in every corridor.
    xy0 = model[:, :2].min(0) - step
    cells = np.floor((model[:, :2] - xy0) / step).astype(int)
    shape = cells.max(0) + 2
    if np.prod(shape) > 2_000_000:
        raise ValueError("Map exceeds the global search grid size")
    tree = cKDTree(ref[:, 1:3])
    z0 = np.median(ref[:, 3])
    bands = np.array([-0.14, 0.12, 0.55, 1.9])
    zb = np.digitize(model[:, 2] - z0, bands) - 1
    qb = np.digitize(query[:, 2], bands) - 1
    maps = []
    for k in range(3):
        grid = np.zeros(shape)
        ix = cells[zb == k]
        grid[tuple(ix.T)] = 1
        maps.append(gaussian_filter(grid, 0.8))
    radius = int(np.ceil(np.linalg.norm(query[:, :2], axis=1).max() / step)) + 1
    hypotheses = []
    for yaw in np.deg2rad(np.arange(0, 360, 10)):
        rot = Rotation.from_euler("z", yaw)
        uv = np.rint(rot.apply(query)[:, :2] / step).astype(int) + radius
        score = np.zeros(shape)
        for k, target in enumerate(maps):
            pattern = np.zeros((2 * radius + 1,) * 2)
            ix = uv[qb == k]
            pattern[tuple(ix.T)] = 1
            score += fftconvolve(target, pattern[::-1, ::-1], mode="same")
        for _ in range(3):
            ij = np.unravel_index(score.argmax(), score.shape)
            xy = xy0 + (np.array(ij) + 0.5) * step
            seed = np.eye(4)
            seed[:3, :3] = rot.as_matrix()
            seed[:2, 3] = xy
            seed[2, 3] = ref[tree.query(xy)[1], 3]
            hypotheses.append((float(score[ij]), seed))
            a, b = ij
            score[max(0, a - 4) : a + 5, max(0, b - 4) : b + 5] = 0
    return sorted(hypotheses, key=lambda item: -item[0])


def match(map_path, path_a, bag, seconds):
    tic = time.monotonic()
    (source, holdout), stamp, used, first_odom = prefix(bag, seconds)
    model = pcd.xyz(map_path)
    ref = read_tum(path_a)
    target = cKDTree(sample(model, 0.065, 900000))
    ranked = []
    for _, seed in candidates(model, source, ref)[:24]:
        pose = fit(source, target, seed)
        ranked.append((distance(holdout, target, pose), pose))
    ranked.sort(key=lambda item: item[0])
    best, pose = ranked[0]
    alt = next((err for err, p in ranked if distinct(pose, p)), None)
    return dict(
        initial_pose=pose7(pose),
        stamp_ns=stamp,
        prefix_s=used,
        first_odometry_pose=first_odom,
        wall_time_s=time.monotonic() - tic,
        heldout_distance_m=best,
        alternate_distance_m=alt,
        separated=bool(alt is not None and alt > 1.25 * best + 0.005 and best < 0.20),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("map", "trajectory_a", "bag_b", "output"):
        parser.add_argument(arg, type=Path)
    parser.add_argument("--seconds", type=float, default=8.0)
    a = parser.parse_args()
    result = match(a.map, a.trajectory_a, a.bag_b, a.seconds)
    write_json(a.output, result)
    print(result)
