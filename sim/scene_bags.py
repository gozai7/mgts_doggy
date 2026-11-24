"""Deterministic planar lidar and local odometry for the Gazebo floor plans."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import rosbag2_py
import yaml
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from world import SCENES, layout_for

DT = 0.2
BEARINGS = np.deg2rad(np.arange(-180, 180))
RANGE = 16.0


def trajectory(route):
    points = np.asarray(route, dtype=float)
    dx, dy = points[1] - points[0]
    yaw = math.atan2(dy, dx)
    yield *points[0], yaw
    for a, b in itertools.pairwise(points):
        dx, dy = b - a
        direction = math.atan2(dy, dx)
        turn = math.remainder(direction - yaw, math.tau)
        steps = math.ceil(abs(turn) / (0.7 * DT))
        for i in range(1, steps + 1):
            yield *a, yaw + turn * i / steps
        yaw = direction
        steps = math.ceil(math.hypot(dx, dy) / (0.52 * DT))
        for i in range(1, steps + 1):
            yield *(a + (b - a) * i / steps), yaw


def intersections(boxes, pos, rays):
    """Nearest forward intersection with axis-aligned boxes and incidence cosine."""
    bounds = np.asarray(boxes)
    lo, hi = bounds[:, :2] - bounds[:, 2:] / 2, bounds[:, :2] + bounds[:, 2:] / 2
    # A parallel ray outside a slab never intersects it.
    parallel = abs(rays) < 1e-12
    denom = np.where(parallel, 1, rays)
    a, b = (lo[:, None] - pos) / denom, (hi[:, None] - pos) / denom
    entry = np.where(parallel, -np.inf, np.minimum(a, b))
    leave = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=2)
    distance = entry.max(axis=2)
    outside = (parallel & ((pos < lo[:, None]) | (pos > hi[:, None]))).any(axis=2)
    valid = ~outside & (distance > 0) & (distance <= leave)
    distance = np.where(valid, distance, np.inf)
    ray = np.arange(len(rays))
    first = distance.argmin(axis=0)
    normal_axis = entry[first, ray].argmax(axis=1)
    return distance[first, ray], abs(rays[ray, normal_axis])


def scan_ranges(layout, pose, rng, glass_rng):
    angles = BEARINGS + pose[2]
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    panels = layout["glass"]
    walls = [b for b in layout["boxes"] if b not in panels]
    ranges, _ = intersections(walls, pose[:2], rays)
    if panels:
        glass, cosine = intersections(panels, pose[:2], rays)
        in_front = (glass < ranges) & (glass <= RANGE)
        # Stress model: return from the panel near its normal, wall behind it, or no echo.
        p_surface = 0.75 * np.exp(-0.5 * (np.arccos(np.clip(cosine, 0, 1)) / np.deg2rad(4)) ** 2)
        u = glass_rng.random(len(rays))
        ranges = np.where(in_front & (u < p_surface), glass, ranges)
        ranges[in_front & (u >= p_surface) & (u < p_surface + 0.2)] = np.inf
    ranges += rng.normal(0, 0.015, len(rays))
    ranges[(ranges > RANGE) | (ranges < 0.12)] = np.inf
    return ranges


def save_reference_map(folder, layout):
    """Rasterize physical obstacles in world coordinates for the oracle-map control."""
    resolution = 0.05
    x0, y0, x1, y1 = layout["bounds"]
    origin = [x0 - 0.5, y0 - 0.5, 0.0]
    x = origin[0] + (np.arange(round((x1 - x0 + 1) / resolution)) + 0.5) * resolution
    y = origin[1] + (np.arange(round((y1 - y0 + 1) / resolution)) + 0.5) * resolution
    xx, yy = np.meshgrid(x, y[::-1])
    grid = np.full(xx.shape, 205, dtype=np.uint8)
    grid[(xx >= x0) & (xx <= x1) & (yy >= y0) & (yy <= y1)] = 254
    for cx, cy, width, depth in layout["boxes"]:
        grid[(abs(xx - cx) <= width / 2) & (abs(yy - cy) <= depth / 2)] = 0
    Image.fromarray(grid).save(folder / "map.pgm")
    config = {
        "image": "map.pgm",
        "resolution": resolution,
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    (folder / "map.yaml").write_text(yaml.safe_dump(config))


def record(bag_path, truth, layout, seed):
    poses = np.asarray(list(trajectory(layout["route"])))
    for x, y, width, depth in layout["boxes"]:
        delta = np.maximum(abs(poses[:, :2] - [x, y]) - [width / 2, depth / 2], 0)
        if np.min(np.linalg.norm(delta, axis=1)) < 0.35:
            raise ValueError("route passes too close to an obstacle")
    rng, glass_rng = (np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(2))
    truth.mkdir()
    (truth / "layout.json").write_text(json.dumps(layout))
    bag = rosbag2_py.SequentialWriter()
    bag.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"), rosbag2_py.ConverterOptions("", "")
    )
    for topic, kind in [("/scan", "sensor_msgs/msg/LaserScan"), ("/odom", "nav_msgs/msg/Odometry")]:
        bag.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format="cdr"))
    scan = LaserScan()
    scan.header.frame_id = "base_footprint"
    scan.angle_min, scan.angle_max, scan.angle_increment = BEARINGS[0], BEARINGS[-1], math.pi / 180
    scan.range_min, scan.range_max, scan.scan_time = 0.12, RANGE, DT
    od = Odometry()
    od.header.frame_id, od.child_frame_id = "odom", "base_footprint"
    ox = oy = heading = 0.0
    with (truth / "trajectory.csv").open("w") as stream:
        table = csv.writer(stream)
        table.writerow(["t", "x", "y", "yaw"])
        for i, pose in enumerate(poses):
            ns = 1_000_000_000 + i * round(DT * 1e9)
            scan.header.stamp = od.header.stamp = Time(nanoseconds=ns).to_msg()
            if i:
                ds = 1.025 * np.linalg.norm(pose[:2] - poses[i - 1, :2])
                ox += ds * math.cos(heading)
                oy += ds * math.sin(heading)
                heading += 1.01 * math.remainder(pose[2] - poses[i - 1, 2], math.tau) + 0.001 * DT
            od.pose.pose.position.x, od.pose.pose.position.y = ox, oy
            q = od.pose.pose.orientation
            q.z, q.w = math.sin(heading / 2), math.cos(heading / 2)
            scan.ranges = scan_ranges(layout, pose, rng, glass_rng).astype("f").tolist()
            bag.write("/odom", serialize_message(od), ns)
            bag.write("/scan", serialize_message(scan), ns)
            table.writerow([ns / 1e9, *pose])
    bag.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=SCENES)
    args = parser.parse_args()
    sessions = [(f"{args.scene}-{letter}", role) for letter, role in [("A", "mapping"), ("B", "inspection")]]
    roots = [Path("/data"), Path("/truth")]
    for root in roots:
        for name, _ in sessions:
            if (root / name).exists():
                parser.error(f"session already exists: {root / name}")
    with TemporaryDirectory(dir=roots[0]) as data, TemporaryDirectory(dir=roots[1]) as truth:
        for seed, (name, role) in enumerate(sessions, 710):
            layout = layout_for(args.scene, role)
            record(Path(data, name), Path(truth, name), layout, seed)
            if role == "mapping":
                save_reference_map(Path(truth, name), layout)
        for root, tmp in zip(roots, [data, truth]):
            for name, _ in sessions:
                Path(tmp, name).rename(root / name)


if __name__ == "__main__":
    main()
