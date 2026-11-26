"""Match a short B submap to a saved A map using only relative B odometry."""

import argparse
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from competition import bagio, cloud, pcd
from competition.cold_start import distinct, fit, moved, sample
from competition.config import pose7, transform, write_json
from competition.trajectory import read_tum


def prefix(bag, seconds, lidar_topic="/utlidar/cloud_base", base_from_lidar=None):
    poses, scans = [], []
    with bagio.reader(bag) as reader:
        topics = [c for c in reader.connections if c.topic in (lidar_topic, "/utlidar/robot_odom")]
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
        if base_from_lidar is not None:
            pts = moved(pts, base_from_lidar)
        sets[i % 2].append(moved(pts, anchor @ poses[j][1]))
    return (
        [sample(np.vstack(s), 0.10, 3200) for s in sets],
        scans[0][0],
        (scans[-1][0] - scans[0][0]) * 1e-9,
        pose7(np.linalg.inv(anchor)),
    )


def candidates(model, query, ref, step=0.20, first_odom=None):
    tilt = Rotation.identity()
    layered = first_odom is not None
    if layered:
        # Remove unknown yaw while preserving measured gravity in the first body frame.
        attitude = Rotation.from_quat(first_odom[3:])
        mat = attitude.as_matrix()
        tilt = Rotation.from_euler("z", -np.arctan2(mat[1, 0], mat[0, 0])) * attitude
        query = tilt.apply(query)
        step = 0.17
    # Work in height bands to avoid voting for floor returns in every corridor.
    xy0 = model[:, :2].min(0) - step
    cells = np.floor((model[:, :2] - xy0) / step).astype(int)
    shape = cells.max(0) + 2
    if np.prod(shape) > 2_000_000:
        raise ValueError("Map exceeds the global search grid size")
    tree = cKDTree(ref[:, 1:3])
    z0 = np.median(ref[:, 3])
    bands = np.array(
        [-0.16, 0.02, 0.22, 0.48, 0.80, 1.30, 2.20] if layered else [-0.14, 0.12, 0.55, 1.9]
    )
    zb = np.digitize(model[:, 2] - z0, bands) - 1
    qb = np.digitize(query[:, 2], bands) - 1
    maps = []
    for k in range(len(bands) - 1):
        grid = np.zeros(shape)
        ix = cells[zb == k]
        grid[tuple(ix.T)] = 1
        if layered and ix.size:
            maps.append(np.exp(-0.5 * (distance_transform_edt(grid == 0) / 1.25) ** 2))
        else:
            maps.append(gaussian_filter(grid, 0.8))
    radius = int(np.ceil(np.linalg.norm(query[:, :2], axis=1).max() / step)) + 1
    hypotheses = []
    for yaw in np.deg2rad(np.arange(0, 360, 5 if layered else 10)):
        rot = Rotation.from_euler("z", yaw)
        uv = np.rint(rot.apply(query)[:, :2] / step).astype(int) + radius
        score = np.zeros(shape)
        for k, target in enumerate(maps):
            pattern = np.zeros((2 * radius + 1,) * 2)
            ix = uv[qb == k]
            pattern[tuple(ix.T)] = 1
            if layered:
                # Sparse walls and shelves must also vote for proposals before ICP.
                pattern /= max(pattern.sum(), 20)
            score += fftconvolve(target, pattern[::-1, ::-1], mode="same")
        for _ in range(6 if layered else 3):
            ij = np.unravel_index(score.argmax(), score.shape)
            xy = xy0 + (np.array(ij) + 0.5) * step
            seed = np.eye(4)
            seed[:3, :3] = (rot * tilt).as_matrix()
            seed[:2, 3] = xy
            seed[2, 3] = z0 if layered else ref[tree.query(xy)[1], 3]
            hypotheses.append((float(score[ij]), seed))
            a, b = ij
            pad = 5 if layered else 4
            score[max(0, a - pad) : a + pad + 1, max(0, b - pad) : b + pad + 1] = 0
    return sorted(hypotheses, key=lambda item: -item[0])


def height_weights(points, first_odom):
    """Balance low obstacles and upper structures when verifying horizontal pose."""
    if len(points) < 60:
        return None
    rotation = transform(first_odom)[:3, :3]
    # Gravity direction is independent of odometry's origin and yaw.
    relative_z = points @ rotation[2]
    band = np.digitize(relative_z, [-0.15, 0.40])
    # Parallel corridor walls constrain the lateral axis only. Balance surface
    # orientations as well as height, so a visible end wall can resolve distance.
    _, idx = cKDTree(points).query(points, k=10)
    offsets = points[idx] - points[idx].mean(axis=1, keepdims=True)
    eig, basis = np.linalg.eigh(offsets.transpose(0, 2, 1) @ offsets)
    normal = basis[:, :, 0]
    level_normal = normal @ rotation.T
    # Direction relative to the first body heading makes the descriptor gauge invariant.
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    heading = (np.arctan2(level_normal[:, 1], level_normal[:, 0]) - yaw) % np.pi
    sector = np.floor(heading * 4 / np.pi).astype(int) % 4
    cell = 4 * band + sector
    structured = (band > 0) & (np.abs(level_normal[:, 2]) < 0.72) & (eig[:, 1] > 2 * eig[:, 0])
    counts = np.bincount(cell[structured], minlength=12)
    weights = structured / np.maximum(counts[cell], 15)
    if np.count_nonzero(weights) < 60:
        return None
    return weights / weights.sum()


def match(map_path, path_a, bag, seconds, lidar_topic="/utlidar/cloud_base", base_from_lidar=None):
    tic = time.monotonic()
    (source, holdout), stamp, used, first_odom = prefix(bag, seconds, lidar_topic, base_from_lidar)
    model = pcd.xyz(map_path)
    ref = read_tum(path_a)
    target = cKDTree(sample(model, 0.065, 900000))
    weights = height_weights(holdout, first_odom) if base_from_lidar is not None else None
    enough_structure = weights is not None or base_from_lidar is None
    ranked = []
    level = first_odom if base_from_lidar is not None else None
    seeds = candidates(model, source, ref, first_odom=level)
    for _, seed in seeds[: 40 if level is not None else 24]:
        pose = fit(source, target, seed)
        distances, _ = target.query(moved(holdout, pose))
        score = float(np.average(np.minimum(distances, 0.60), weights=weights))
        ranked.append((score, pose))
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
        score_weighting="height_and_surface_orientation" if weights is not None else "uniform",
        heldout_structured_points=int(np.count_nonzero(weights)) if weights is not None else None,
        separated=bool(
            enough_structure and alt is not None and alt > 1.25 * best + 0.005 and best < 0.20
        ),
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
