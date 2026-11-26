#!/usr/bin/env python3
"""Export the Go2 recording's world-frame A clouds and onboard A trajectory."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition import bagio, cloud
from competition.trajectory import save


def export(bag, output):
    clouds, trajectory = [], []
    with bagio.reader(bag) as reader:
        selected = [c for c in reader.connections if c.topic in ("/utlidar/cloud_deskewed", "/utlidar/robot_odom")]
        for conn, _, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, conn.msgtype)
            if msg.header.frame_id != "odom":
                raise ValueError(f"{conn.topic} must be expressed in odom")
            if conn.topic == "/utlidar/cloud_deskewed":
                xyz, _ = cloud.unpack(msg)
                valid = np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
                clouds.append(xyz[valid])
            else:
                if msg.child_frame_id != "base_link":
                    raise ValueError("Go2 odometry must describe base_link")
                trajectory.append([bagio.ns(msg) * 1e-9, *bagio.pose_values(msg.pose.pose)])
    if not clouds or len(trajectory) < 2 or not np.any([len(points) for points in clouds]):
        raise ValueError("The recording must contain nonempty Go2 world clouds and odometry")
    poses = np.asarray(trajectory)
    if not np.isfinite(poses).all() or not (np.diff(poses[:, 0]) > 0).all() or np.any(
        abs(np.linalg.norm(poses[:, 4:], axis=1) - 1.0) > .01
    ):
        raise ValueError("Invalid Go2 odometry poses or timestamps")

    points = np.concatenate(clouds)
    leaf = .05
    _, voxel, counts = np.unique(np.floor(points / leaf).astype(np.int64), axis=0,
                                 return_inverse=True, return_counts=True)
    centers = np.column_stack([np.bincount(voxel, weights=axis) / counts for axis in points.T]).astype("<f4")
    output.mkdir(parents=True, exist_ok=False)
    count = len(centers)
    header = (f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {count}\nHEIGHT 1\nPOINTS {count}\nDATA binary\n")
    with (output / "map.pcd").open("wb") as dst:
        dst.write(header.encode("ascii"))
        dst.write(centers.tobytes())
    save(output / "trajectory.tum", poses, csv=False)
    (output / "source.json").write_text(json.dumps({
        "bag": str(bag), "frame": "odom", "map_topic": "/utlidar/cloud_deskewed",
        "trajectory_topic": "/utlidar/robot_odom", "voxel_m": leaf, "map_points": count,
    }, indent=2) + "\n")
    return count


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(f"Exported {export(args.bag, args.output)} points to {args.output}")
