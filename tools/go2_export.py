"""Write the measured A map and B trajectory as a replayable ROS 2 result bag."""

import argparse
import csv
from pathlib import Path

from competition import bagio, cloud, pcd
from competition.trajectory import read_tum
from competition.go2_full_route import path_due
import numpy as np


def export(root):
    track = read_tum(root / "trajectory.tum")
    T = bagio.STORE.types

    sources = []
    prefix_stamps = []
    if (root / "tracking.csv").exists():
        with (root / "tracking.csv").open() as f:
            for record in csv.DictReader(f):
                n = int(record["stamp_ns"])
                if record["source"] in ("lidar", "odometry"):
                    sources.append((n, record["source"]))
                if record.get("full_route_source") == "registered_odometry":
                    prefix_stamps.append(n)
        if len(sources) != len(track):
            raise ValueError("Tracking source count differs from trajectory length")

    full = read_tum(root / "full_route.tum") if (root / "full_route.tum").is_file() else None
    if full is not None:
        n_prefix = len(full) - len(track)
        if n_prefix < 0 or not np.array_equal(full[n_prefix:], track):
            raise ValueError("Full route must end with the unchanged online trajectory")
        if not (root / "tracking.csv").exists():
            prefix_stamps = [round(row[0] * 1e9) for row in full[:n_prefix]]
        if len(prefix_stamps) != n_prefix or any(
            abs(n * 1e-9 - row[0]) > 5e-7 for n, row in zip(prefix_stamps, full[:n_prefix])
        ):
            raise ValueError("Full route prefix timestamps differ from tracking history")

    def stamped_pose(row, header):
        pose = T["geometry_msgs/msg/Pose"](
            T["geometry_msgs/msg/Point"](*row[1:4]),
            T["geometry_msgs/msg/Quaternion"](*row[4:8]),
        )
        return T["geometry_msgs/msg/PoseStamped"](header, pose)

    def messages():
        poses = []
        full_poses = [] if full is None else [
            stamped_pose(row, bagio.header(n, "map")) for n, row in zip(prefix_stamps, full)
        ]
        last_full_stamp = None
        for i, row in enumerate(track):
            stamp = sources[i][0] if sources else round(row[0] * 1e9)
            header = bagio.header(stamp, "map")
            if i == 0:
                yield (
                    "/competition/result/map",
                    stamp,
                    cloud.pack(pcd.xyz(root / "map.pcd"), stamp, "map"),
                )
            stamped = stamped_pose(row, header)
            poses.append(stamped)
            yield "/competition/result/base_pose", stamp, stamped
            if sources:
                yield "/competition/result/source", stamp, T["std_msgs/msg/String"](sources[i][1])
            if full is not None:
                full_poses.append(stamped)
                if path_due(stamp, last_full_stamp) or i == len(track) - 1:
                    yield "/competition/result/full_route", stamp, T["nav_msgs/msg/Path"](header, list(full_poses))
                    last_full_stamp = stamp
            if i % 15 == 0 or i == len(track) - 1:
                yield (
                    "/competition/result/localization",
                    stamp,
                    T["nav_msgs/msg/Path"](header, list(poses)),
                )

        # Complete offline routes are published only after the last online update.
        if (root / "glim_b_in_map.tum").is_file():
            offline_b = read_tum(root / "glim_b_in_map.tum")
            offline_a = read_tum(root / "mapping.tum")
            end = max(stamp, round(offline_b[-1, 0] * 1e9), round(offline_a[-1, 0] * 1e9))
            for name, route in (("mapping", offline_a), ("glim_b", offline_b)):
                full_path = [
                    stamped_pose(row, bagio.header(round(row[0] * 1e9), "map")) for row in route
                ]
                yield (
                    "/competition/result/" + name,
                    end,
                    T["nav_msgs/msg/Path"](bagio.header(end, "map"), full_path),
                )

    bagio.write_messages(root / "result_bag", messages())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    export(parser.parse_args().results)
