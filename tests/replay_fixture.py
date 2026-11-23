"""Tiny analytic bag pair for offline regression tests without a GPU or Gazebo."""

import csv
import json
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


def fixture(session, start, speed):
    data = Path("/data", session)
    truth = Path("/truth", session)
    truth.mkdir(exist_ok=False)
    bag = rosbag2_py.SequentialWriter()
    bag.open(rosbag2_py.StorageOptions(uri=str(data), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    scan_topic = "/inspection/laser" if session.endswith("B") else "/scan"
    for topic, kind in [
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ("/odom", "nav_msgs/msg/Odometry"),
        ("/imu/data", "sensor_msgs/msg/Imu"),
        (scan_topic, "sensor_msgs/msg/LaserScan"),
    ]:
        bag.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format="cdr"))
    with (truth / "trajectory.csv").open("w") as stream:
        csvout = csv.writer(stream)
        csvout.writerow(["t", "x", "y", "yaw"])
        for i in range(80):
            ns = 1_000_000_000 + i * 100_000_000
            when = Time(nanoseconds=ns).to_msg()
            imu = Imu()
            imu.header.stamp = when
            bag.write("/imu/data", serialize_message(imu), ns)
            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = when, "odom", "base_footprint"
            od.pose.pose.position.x = speed * i * 1.06
            od.pose.pose.orientation.w = 1.0
            bag.write("/odom", serialize_message(od), ns)
            if i == 0:
                transforms = []
                for parent, child, x in [
                    ("base_footprint", "base_link", 0.0),
                    ("base_link", "trunk", 0.0),
                    ("trunk", "scan_link", 0.0),
                    ("map", "odom", 100.0),
                ]:
                    tf = TransformStamped()
                    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = when, parent, child
                    tf.transform.translation.x, tf.transform.rotation.w = x, 1.0
                    transforms.append(tf)
                bag.write("/tf_static", serialize_message(TFMessage(transforms=transforms)), ns)
            x = start + i * speed
            csvout.writerow([ns * 1e-9, x, 0.0, 0.0])
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = "scan_link", when
            scan.angle_min, scan.angle_increment = -np.pi, np.pi / 180
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min, scan.range_max = 0.12, 16.0
            angles = scan.angle_min + np.arange(360) * scan.angle_increment
            with np.errstate(divide="ignore"):
                rx = np.where(np.cos(angles) > 0, 6 - x, -3 - x) / np.cos(angles)
                ry = np.where(np.sin(angles) > 0, 2.4, -2) / np.sin(angles)
            scan.ranges = np.minimum(rx, abs(ry)).tolist()
            bag.write(scan_topic, serialize_message(scan), ns)
    profile = yaml.safe_load(Path("/mgts/configs/go2.yaml").read_text())
    profile["lidar"] = scan_topic
    (data / "profile.yaml").write_text(yaml.safe_dump(profile))
    (truth / "layout.json").write_text(
        json.dumps(
            dict(
                bounds=[-3, -2, 6, 2.4],
                boxes=[[-3, 0.2, 0.1, 4.4], [6, 0.2, 0.1, 4.4], [1.5, -2, 9, 0.1], [1.5, 2.4, 9, 0.1]],
            )
        )
    )


if __name__ == "__main__":
    fixture("fixture-A", 0.0, 0.04)
    fixture("fixture-B", 1.0, 0.03)
