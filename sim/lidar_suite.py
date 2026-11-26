"""Render independent 3D LiDAR/IMU bags and survey references for indoor GLIM tests."""

import argparse
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from zlib import crc32

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
from competition import bagio, cloud
from competition.config import load, write_json
from competition.occupancy import export_grid
from competition.trajectory import save

PLANS = Path(__file__).with_name("lidar_scenes")
BASE_Z, LIDAR_Z, RANGE = 0.45, 0.65, 10.0
angles, elevation = np.meshgrid(
    np.deg2rad(np.arange(0, 360, 2)), np.deg2rad([-26, -18, -11, -5, 1, 8, 16, 28])
)
BEAMS = np.stack(
    (np.cos(elevation) * np.cos(angles), np.cos(elevation) * np.sin(angles), np.sin(elevation)),
    axis=-1,
).reshape(-1, 3)


def surfaces(plan):
    x0, y0, x1, y1 = plan["bounds"]
    z = plan["ceiling"]
    perimeter = [[x, (y0 + y1) / 2, 0.12, y1 - y0 + 0.12, 0.0, z] for x in (x0, x1)]
    perimeter += [[(x0 + x1) / 2, y, x1 - x0 + 0.12, 0.12, 0.0, z] for y in (y0, y1)]
    return perimeter + plan["boxes"], plan.get("glass", [])


def limits(box):
    x, y, w, d, lo, hi = box
    return np.array([x - w / 2, y - d / 2, lo]), np.array([x + w / 2, y + d / 2, hi])


def hits(boxes, origin, inverse):
    ranges = np.full(len(inverse), np.inf)
    for low, high in boxes:
        a, b = (low - origin) * inverse, (high - origin) * inverse
        entry = np.minimum(a, b).max(axis=1)
        exit_ = np.maximum(a, b).min(axis=1)
        np.minimum(ranges, np.where((entry > 0) & (exit_ >= entry), entry, np.inf), out=ranges)
    return ranges


def scan(pose, solid, glass, ceiling, rng=None):
    x, y, yaw = pose
    rot = Rotation.from_euler("z", yaw)
    rays = rot.apply(BEAMS)
    origin = np.array([x, y, LIDAR_Z])
    inverse = 1 / np.where(abs(rays) < 1e-12, 1e-12, rays)
    horizontal = np.where(rays[:, 2] > 0, ceiling - LIDAR_Z, -LIDAR_Z) / rays[:, 2]
    dist = np.minimum(horizontal, hits(solid, origin, inverse))
    if glass:
        panel = hits(glass, origin, inverse)
        front = panel < dist
        # Simple stress model per beam: surface echo, lost return or transmission.
        u = rng.random(len(dist)) if rng is not None else np.zeros(len(dist))
        dist = np.where(front & (u < 0.30), panel, dist)
        dist[front & (u >= 0.30) & (u < 0.45)] = np.inf
    valid = (dist >= 0.3) & (dist <= RANGE)
    if rng is not None:
        valid &= rng.random(len(dist)) > 0.008
        dist = dist + rng.normal(0, 0.012, len(dist))
    return BEAMS[valid] * dist[valid, None]


def path_samples(route):
    points = np.asarray(route, dtype=float)
    dx, dy = np.diff(points, axis=0).T
    headings = np.arctan2(dy, dx)
    pose = np.r_[points[0], headings[0]]
    samples = [np.tile(pose, (301, 1))]  # Three seconds for IMU initialization.
    for point, heading in zip(points[1:], headings):
        turn = math.remainder(heading - pose[2], 2 * np.pi)
        for delta, speed in (
            (np.array([0.0, 0.0, turn]), 0.95),
            (np.r_[point - pose[:2], 0.0], 1.2),
        ):
            length = np.linalg.norm(delta)
            if length > 1e-9:
                steps = math.ceil(150 * length / speed)
                u = np.arange(1, steps + 1) / steps
                samples.append(pose + (u * u * (3 - 2 * u))[:, None] * delta)
                pose = pose + delta
    samples.append(np.tile(pose, (101, 1)))
    return np.concatenate(samples)


def record(dest, survey, phase, route, ceiling, solid, glass, seed):
    poses = path_samples(route)
    xy = poses[:, :2]
    for lo, hi in solid + glass:
        if lo[2] <= BASE_Z <= hi[2]:
            gap = np.maximum(np.maximum(lo[:2] - xy, xy - hi[:2]), 0.0)
            if np.linalg.norm(gap, axis=1).min() < 0.3:
                raise ValueError(f"{phase} route collides with {lo}, {hi}")
    rotation = Rotation.from_euler("z", poses[:, 2])
    acc = np.gradient(np.gradient(xy, 0.01, axis=0), 0.01, axis=0)
    acc = rotation.inv().apply(np.column_stack((acc, np.full(len(acc), 9.80665))))
    gyro = np.gradient(poses[:, 2], 0.01)
    scan_rng, imu_rng = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(2)]
    acc += imu_rng.normal(0, 0.018, acc.shape)
    gyro += imu_rng.normal(0, 0.0015, len(gyro))
    start = (1700000000 if phase == "mapping" else 1700010000) * 10**9
    T = bagio.STORE.types
    vec = T["geometry_msgs/msg/Vector3"]
    quat = T["geometry_msgs/msg/Quaternion"](0.0, 0.0, 0.0, 1.0)

    def messages():
        for i, pose in enumerate(poses):
            stamp = start + i * 10_000_000
            imu = T["sensor_msgs/msg/Imu"](
                bagio.header(stamp, "imu"),
                quat,
                np.r_[-1.0, np.zeros(8)],
                vec(0.0, 0.0, gyro[i]),
                np.diag([0.0015**2] * 3).ravel(),
                vec(*acc[i]),
                np.diag([0.018**2] * 3).ravel(),
            )
            yield "/imu/data", stamp, imu
            if i % 10 == 0:
                xyz = scan(pose, solid, glass, ceiling, scan_rng)
                yield "/lidar/points", stamp, cloud.pack(xyz, stamp, "lidar")

    bagio.write_messages(dest / phase, messages())
    ids = np.arange(0, len(poses), 10)
    truth = np.column_stack(
        (start * 1e-9 + ids * 0.01, xy[ids], np.full(len(ids), BASE_Z), rotation.as_quat()[ids])
    )
    save(survey / (phase + ".tum"), truth, csv=False)
    return {
        "scans": len(ids),
        "imu": len(poses),
        "length_m": float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
        "start_world": truth[0, 1:].tolist(),
    }


def reference_maps(folder, plan, boxes):
    x0, y0, x1, y1 = plan["bounds"]
    z = plan["ceiling"]
    resolution = 0.05
    origin = np.array([x0 - 0.1, y0 - 0.1])
    xx, yy = np.meshgrid(
        np.arange(origin[0] + resolution / 2, x1 + 0.1, resolution),
        np.arange(origin[1] + resolution / 2, y1 + 0.1, resolution),
    )
    grid = np.zeros(xx.shape, np.int8)
    grid[(xx < x0) | (xx > x1) | (yy < y0) | (yy > y1)] = -1
    for lo, hi in boxes:
        if lo[2] <= 1.8 and hi[2] >= 0.1:
            grid[(xx >= lo[0]) & (xx <= hi[0]) & (yy >= lo[1]) & (yy <= hi[1])] = 100
    export_grid(folder, grid, origin, resolution, 0, "world")
    faces = []
    surfaces_to_sample = [(lo, hi, range(3)) for lo, hi in boxes]
    surfaces_to_sample.append((np.array([x0, y0, 0.0]), np.array([x1, y1, z]), [2]))
    for lo, hi, axes in surfaces_to_sample:
        for axis in axes:
            a, b = [i for i in range(3) if i != axis]
            lines = [
                np.linspace(lo[i], hi[i], math.ceil((hi[i] - lo[i]) / 0.12) + 1) for i in (a, b)
            ]
            u, v = np.meshgrid(*lines)
            for edge in (lo[axis], hi[axis]):
                f = np.empty((u.size, 3))
                f[:, a], f[:, b], f[:, axis] = u.ravel(), v.ravel(), edge
                faces.append(f)
    pts = np.unique(np.round(np.concatenate(faces), 5), axis=0).astype("<f4")
    hdr = f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\nPOINTS {len(pts)}\nDATA binary\n"
    (folder / "map.pcd").write_bytes(hdr.encode() + pts.tobytes())


def generate(root, name, seed):
    root.mkdir(parents=True, exist_ok=True)
    if (root / name).exists():
        raise FileExistsError(root / name)
    plan = yaml.safe_load((PLANS / (name + ".yaml")).read_text())
    walls, panels = surfaces(plan)
    solid, glass = [limits(b) for b in walls], [limits(b) for b in panels]
    with TemporaryDirectory(dir=root) as temp:
        case = Path(temp) / name
        data, survey = case / "data", case / "reference"
        data.mkdir(parents=True)
        survey.mkdir()
        reference_maps(survey, plan, solid + glass)
        manifest = {"scene": name, "seed": seed, "frame": "world"}
        # Use identical noise in the paired galleries to isolate the landmark.
        family = "gallery-pair" if name in ("landmark-gallery", "symmetric-gallery") else name
        for i, (phase, route) in enumerate(
            (("mapping", "mapping"), ("localization", "inspection"))
        ):
            manifest[phase] = record(
                data,
                survey,
                phase,
                plan["routes"][route],
                plan["ceiling"],
                solid,
                glass,
                [seed, crc32(family.encode()), i],
            )
        write_json(survey / "manifest.json", manifest)
        config = load(ROOT / "configs/competition/dataset.yaml")
        config["output"] = "/results/lidar-scenes/" + name
        config["topics"].update(lidar="/lidar/points", imu="/imu/data")
        config["extrinsics"].update(
            T_base_lidar=[0.0, 0.0, LIDAR_Z - BASE_Z, 0.0, 0.0, 0.0, 1.0],
            T_base_imu=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        config["units"].update(acceleration="m/s2", angular_velocity="rad/s")
        config["point_time"]["mode"] = "none"
        config["mapping"]["mode"] = "gpu"
        config["occupancy"]["floor_z_m"] = -LIDAR_Z  # GLIM anchors this map at the first lidar pose.
        (data / "dataset.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
        case.rename(root / name)
    print(
        f"{name}: {manifest['mapping']['scans']} + {manifest['localization']['scans']} scans",
        flush=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    scenes = sorted(p.stem for p in PLANS.glob("*.yaml"))
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", choices=scenes, action="append")
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    for name in args.scene or scenes:
        generate(args.output, name, args.seed)
