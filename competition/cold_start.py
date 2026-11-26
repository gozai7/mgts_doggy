"""Estimate map <- first base from the A map and normalized B scans, before NDT replay."""
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from . import bagio, cloud, pcd
from .config import pose7, transform
from .trajectory import interpolate, read_tum

# Use the same BBS implementation as the team's ROS global-localization node.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor/lidar_localization_ros2/scripts"))
from make_bbs_relocalization_attempts import branch_and_bound_candidates, voxel_downsample_2d


def moved(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def sample(points, leaf, limit):
    _, ids = np.unique(np.floor(points / leaf).astype(np.int64), axis=0, return_index=True)
    return points[ids[::max(1, int(np.ceil(len(ids) / limit)))]]


def fit(points, target, initial):
    pose = initial.copy()
    for _ in range(24):
        distances, ids = target.query(moved(points, pose))
        keep = distances < .55
        if np.count_nonzero(keep) < 12:
            break
        a, b = points[keep], target.data[ids[keep]]
        u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
        vt[-1] *= np.linalg.det(vt.T @ u.T)
        rot = vt.T @ u.T
        update = np.eye(4)
        update[:3, :3], update[:3, 3] = rot, b.mean(0) - rot @ a.mean(0)
        change = np.max(np.abs(update - pose))
        pose = update
        if change < .0002:
            break
    return pose


def distance(points, tree, pose):
    distances, _ = tree.query(moved(points, pose))
    return float(np.minimum(distances, .60).mean())


def distinct(first, second):
    relative = first[:3, :3].T @ second[:3, :3]
    return (np.linalg.norm(first[:3, 3] - second[:3, 3]) > .6
            or Rotation.from_matrix(relative).magnitude() > np.deg2rad(20))


def initialize(map_path, bag, config, reference_path=None):
    model = pcd.xyz(map_path)
    floor = config["occupancy"]["floor_z_m"]
    low, high = np.asarray(config["occupancy"]["height_m"]) + floor
    res = .25
    band = model[(model[:, 2] >= low) & (model[:, 2] <= high), :2]
    if len(band) < 12:
        raise ValueError("Insufficient map geometry in the localization height band")
    # Pad one coarsest BBS block to preserve hypotheses near the map boundary.
    block = 2**4
    origin = band.min(0) - block * res
    cells = np.floor((band - origin) / res).astype(int)
    shape = cells.max(0)[::-1] + block + 1
    if int(shape[0]) * int(shape[1]) > config["occupancy"]["max_cells"]:
        raise ValueError("Cold-start search exceeds the configured map size limit")
    grid = np.zeros(shape, bool)
    grid[cells[:, 1], cells[:, 0]] = True
    grid = binary_dilation(grid)
    target = cKDTree(sample(model, .08, 2_000_000))
    ref = read_tum(reference_path) if reference_path is not None else None
    ref_xy = cKDTree(ref[:, 1:3]) if ref is not None else None
    nominal_z = float(np.median(ref[:, 3])) if ref is not None else 0.
    mount = transform(config["extrinsics"]["T_base_lidar"])
    first_from_current = np.eye(4)
    previous = None
    last_stamp = None
    attempts = []
    query_after = 1.
    with bagio.reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic == "/competition/lidar"]
        odometry = None
        if config["localization"]["prediction"] == "kinematic":
            odometry_conns = [c for c in reader.connections if c.topic == "/competition/odometry"]
            odometry = []
            for conn, _, data in reader.messages(connections=odometry_conns):
                msg = reader.deserialize(data, conn.msgtype)
                odometry.append([bagio.ns(msg) * 1e-9, *bagio.pose_values(msg.pose.pose)])
            odometry = np.asarray(odometry)
            if len(odometry) < 2:
                raise ValueError("Automatic initialization requires normalized B odometry")
        for conn, _, data in reader.messages(connections=conns):
            msg = reader.deserialize(data, conn.msgtype)
            stamp = bagio.ns(msg)
            if last_stamp is not None and stamp - last_stamp < 250_000_000:
                continue
            if odometry is not None:
                time_s = stamp * 1e-9
                period = config["lidar"]["scan_period_s"]
                if previous is None and 0 < odometry[0, 0] - time_s <= period:
                    time_s = odometry[0, 0]
                pose_at_scan, valid = interpolate(odometry, [time_s], max_gap=2 * period)
                if not valid[0]:
                    if previous is None:
                        raise ValueError("B odometry does not cover the first LiDAR scan")
                    continue
                odom_from_current = transform(pose_at_scan[0])
            points = sample(moved(cloud.unpack(msg)[0], mount), .12, 1800)
            if len(points) < 12:
                raise ValueError("Insufficient B scan geometry for automatic initialization")
            if previous is None:
                first_stamp = last_stamp = stamp
                accumulated, previous = points, cKDTree(points)
                if odometry is not None:
                    first_from_odom = np.linalg.inv(odom_from_current)
                    odom_origin_hint = odom_from_current if config["localization"].get("use_odom_origin_hint") else None
                continue
            if odometry is not None:
                first_from_current = first_from_odom @ odom_from_current
            else:
                step = fit(points, previous, np.eye(4))
                if distance(points, previous, step) > .24:
                    raise ValueError("Cannot link successive B scans for automatic initialization")
                first_from_current = first_from_current @ step
            held_out = moved(points, first_from_current)
            elapsed = (stamp - first_stamp) * 1e-9
            if elapsed >= query_after:
                selected = accumulated[(accumulated[:, 2] >= low - nominal_z)
                                       & (accumulated[:, 2] <= high - nominal_z), :2]
                xy = voxel_downsample_2d(selected, res)
                xy = xy[::max(1, int(np.ceil(len(xy) / 512)))]
                candidates = branch_and_bound_candidates(grid, xy, res, np.deg2rad(5), 4, 32)
                # Preserve heading alternatives at one location and spatially separated hypotheses.
                spaced = branch_and_bound_candidates(grid, xy, res, np.deg2rad(5), 4, 8, 4)
                candidates = dict.fromkeys([*candidates, *spaced])
                seeds = []
                for candidate in candidates:
                    seed = np.eye(4)
                    seed[:2, 3] = origin + (np.array([candidate.tx_cell, candidate.ty_cell]) + .5) * res
                    seed[2, 3] = ref[ref_xy.query(seed[:2, 3])[1], 3] if ref is not None else nominal_z
                    seed[:3, :3] = Rotation.from_euler("z", candidate.yaw_rad).as_matrix()
                    seeds.append(seed)
                if odometry is not None and odom_origin_hint is not None:
                    # The recordings may have different odom origins. Score this hint like any map hypothesis.
                    seeds.append(odom_origin_hint)
                ranked = []
                for seed in seeds:
                    pose = fit(accumulated, target, seed)
                    cost = max(distance(accumulated, target, pose), distance(held_out, target, pose))
                    ranked.append((cost, pose))
                ranked.sort(key=lambda pair: pair[0])
                if ranked:
                    best, pose = ranked[0]
                    alternate = next((score for score, p in ranked if distinct(pose, p)), None)
                    attempts.append(dict(elapsed_s=elapsed, error_m=best, alternate_error_m=alternate))
                    if best < .20 and alternate is not None and alternate > 1.25 * best + .005:
                        return dict(initial_pose=pose7(pose), stamp_ns=first_stamp, attempts=attempts)
                query_after += min(15., max(2., elapsed))
            accumulated = sample(np.vstack([accumulated, held_out]), .14, 2400)
            previous, last_stamp = cKDTree(points), stamp
    raise ValueError(f"No unambiguous automatic B initialization; search history: {attempts}")
