"""Write the measured A map and B trajectory as a replayable ROS 2 result bag."""

import argparse
from pathlib import Path

from competition import bagio, cloud, pcd
from competition.trajectory import read_tum


def export(root):
    track = read_tum(root / "trajectory.tum")
    T = bagio.STORE.types

    def messages():
        poses = []
        for i, row in enumerate(track):
            stamp = round(row[0] * 1e9)
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
