"""Write the measured A map and B trajectory as a replayable ROS 2 result bag."""

import argparse
import csv
from pathlib import Path

from competition import bagio, cloud, pcd
from competition.trajectory import read_tum


def export(root):
    track = read_tum(root / "trajectory.tum")
    T = bagio.STORE.types

    sources = []
    if (root / "tracking.csv").exists():
        with (root / "tracking.csv").open() as f:
            sources = [
                (int(r["stamp_ns"]), r["source"])
                for r in csv.DictReader(f)
                if r["source"] in ("lidar", "odometry")
            ]
        if len(sources) != len(track):
            raise ValueError("Tracking source count differs from trajectory length")

    def messages():
        poses = []
        for i, row in enumerate(track):
            stamp = sources[i][0] if sources else round(row[0] * 1e9)
            header = bagio.header(stamp, "map")
            if i == 0:
                yield (
                    "/competition/result/map",
                    stamp,
                    cloud.pack(pcd.xyz(root / "map.pcd"), stamp, "map"),
                )
            pose = T["geometry_msgs/msg/Pose"](
                T["geometry_msgs/msg/Point"](*row[1:4]),
                T["geometry_msgs/msg/Quaternion"](*row[4:8]),
            )
            stamped = T["geometry_msgs/msg/PoseStamped"](header, pose)
            poses.append(stamped)
            yield "/competition/result/base_pose", stamp, stamped
            if sources:
                yield "/competition/result/source", stamp, T["std_msgs/msg/String"](sources[i][1])
            if i % 15 == 0 or i == len(track) - 1:
                yield (
                    "/competition/result/localization",
                    stamp,
                    T["nav_msgs/msg/Path"](header, list(poses)),
                )

    bagio.write_messages(root / "result_bag", messages())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    export(parser.parse_args().results)
