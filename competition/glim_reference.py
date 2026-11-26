"""Place GLIM's native map and optimized trajectory in a single level frame."""

import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from . import bagio, cloud, pcd
from .cold_start import moved, sample
from .config import transform, pose7, write_json
from .trajectory import interpolate, read_tum, save
from .glim_refine import refine


def accumulate_scans(bag, poses, mount):
    """Deskew normalized lidar returns using the optimized GLIM body poses."""
    scans = []
    with bagio.reader(bag) as reader:
        channels = [c for c in reader.connections if c.topic == "/competition/lidar"]
        for channel, _, data in reader.messages(connections=channels):
            msg = reader.deserialize(data, channel.msgtype)
            points, fields = cloud.unpack(msg)
            times = bagio.ns(msg) * 1e-9 + fields["time"].astype(np.float64)
            left = max(0, np.searchsorted(poses[:, 0], times.min()) - 1)
            right = np.searchsorted(poses[:, 0], times.max()) + 1
            block = poses[left:right]
            if len(block) < 2:
                continue
            trajectory, covered = interpolate(block, times, max_gap=0.15)
            if covered.any():
                xyz = moved(points[covered], mount)
                q = Rotation.from_quat(trajectory[covered, 3:])
                scans.append(q.apply(xyz) + trajectory[covered, :3])
    if not scans:
        raise ValueError("No raw A returns overlap the optimized GLIM trajectory")
    return sample(np.vstack(scans), 0.055, 1000000).astype("<f4")


def export(folder, dest):
    folder = Path(folder)
    receipt = json.loads((folder / "complete.json").read_text())
    if not receipt["complete"] or receipt["backend"] != "glim":
        raise ValueError("A completed GLIM mapping run is required")
    poses = read_tum(folder / "trajectory_optimized.tum")
    # Level the whole GLIM solution before finding vertical planes.
    anchor, valid = interpolate(read_tum(folder / "kinematic.tum"), poses[:1, 0])
    if not valid[0]:
        raise ValueError("Missing odometry at the GLIM frame origin")
    basis = transform(anchor[0]) @ np.linalg.inv(transform(poses[0, 1:]))
    poses[:, 1:4] = moved(poses[:, 1:4], basis)
    poses[:, 4:] = (
        Rotation.from_matrix(basis[:3, :3]) * Rotation.from_quat(poses[:, 4:])
    ).as_quat()
    config = yaml.safe_load((folder / "config_resolved.yaml").read_text())
    bag = folder.parent / "normalized/mapping/bag"
    mount = transform(config["extrinsics"]["T_base_lidar"])
    poses, refinement = refine(bag, poses, mount)
    save(dest / "mapping.tum", poses, csv=False)
    write_json(dest / "refinement.json", refinement)
    points = accumulate_scans(bag, poses, mount)
    header = f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    (dest / "map.pcd").write_bytes(header.encode() + points.tobytes())
    write_json(
        dest / "mapping_source.json",
        dict(
            backend="glim",
            source=str(folder),
            T_display_glim=pose7(basis),
            points=len(points),
            native_map_points=len(pcd.xyz(folder / "map.pcd")),
            cloud_source="raw returns deskewed along mapping.tum",
            refinement_report="refinement.json",
            kinematic_factors=int((folder / "dump/go2_odom_factors.txt").read_text()),
        ),
    )
