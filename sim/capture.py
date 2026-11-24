"""Collect one independent Go2 session. Run teleop in a second terminal."""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from world import SCENES, layout_for, write_world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=["mapping", "inspection"])
    parser.add_argument("session")
    parser.add_argument("--scene", choices=["office", *SCENES], default="office")
    parser.add_argument("--cloud", action="store_true")
    parser.add_argument("--spawn", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    args = parser.parse_args()
    if Path(args.session).name != args.session or args.session in ("", ".", ".."):
        parser.error("session must be a directory name")
    data, truth = [Path(root, args.session) for root in ("/data", "/truth")]
    if data.exists() or truth.exists():
        parser.error("session already exists")
    data.mkdir()
    truth.mkdir()
    layout = layout_for(args.scene, args.role)
    if args.spawn is None:
        route = layout["route"]
        if route:
            (x, y), (nx, ny) = route[:2]
            args.spawn = [x, y, math.atan2(ny - y, nx - x)]
        else:
            args.spawn = [0.0, 0.0, 0.0]
    world_file = truth / f"{args.scene}.sdf"
    write_world(
        Path(get_package_share_directory("unitree_go2_description"), "worlds/default.sdf"),
        layout,
        world_file,
    )
    (truth / "layout.json").write_text(json.dumps(layout))
    (truth / "spawn.json").write_text(json.dumps(args.spawn))
    lidar = "/velodyne_points/points" if args.cloud else "/scan"
    manifest = {
        "role": args.role,
        "scene": args.scene,
        "lidar": lidar,
        "revision": "55ff151632a0220bf3ae0c0772392615d86da7a8",
        "complete": False,
    }
    manifest_path = data / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    rclpy.init()
    node = rclpy.create_node("capture_monitor")
    processes = []
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    def launch(*command):
        processes.append(subprocess.Popen(command, start_new_session=True))

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if os.environ.get("MGTS_HEADLESS") == "1":
        os.environ["MGTS_GZ_ARGS"] = " -s -r --headless-rendering"
    seen = set()
    required = {lidar, "/imu/data", "/odom", "/tf", "/tf_static", "truth"}
    samples = 0
    recording = False
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        with (truth / "trajectory.csv").open("x") as stream:
            writer = csv.writer(stream)
            writer.writerow(["t", "x", "y", "yaw"])

            def measure(msg):
                nonlocal samples
                seen.add("truth")
                if not recording:
                    return
                pose = msg.pose.pose
                q = pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                stamp = msg.header.stamp
                writer.writerow([f"{stamp.sec}.{stamp.nanosec:09}", pose.position.x, pose.position.y, yaw])
                samples += 1

            node.create_subscription(Odometry, "/odom/ground_truth", measure, qos_profile_sensor_data)
            for topic, msg_type in [
                (lidar, PointCloud2 if args.cloud else LaserScan),
                ("/imu/data", Imu),
                ("/odom", Odometry),
                ("/tf", TFMessage),
                ("/tf_static", TFMessage),
            ]:
                qos = (
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    if topic == "/tf_static"
                    else qos_profile_sensor_data
                )
                node.create_subscription(msg_type, topic, lambda _, t=topic: seen.add(t), qos)
            if not args.cloud:
                launch(
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                )
            x, y, yaw = args.spawn
            launch(
                "ros2",
                "launch",
                "unitree_go2_sim",
                "unitree_go2_launch.py",
                f"world:={world_file}",
                "unitree_go2_description_path:=/mgts/sim/go2.xacro",
                "rviz:=false",
                "disable_d455:=true",
                "disable_camera:=true",
                "disable_lidar_l1:=true",
                f"disable_velodyne_lidar:={str(not args.cloud).lower()}",
                f"world_init_x:={x}",
                f"world_init_y:={y}",
                f"world_init_heading:={yaw}",
            )
            pending = None
            next_check = 0.0
            started = time.monotonic()
            while not stopped:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("a capture process exited unexpectedly")
                rclpy.spin_once(node, timeout_sec=0.2)
                now = time.monotonic()
                if recording:
                    continue
                if seen == required and controllers.service_is_ready() and now >= next_check:
                    if pending is None:
                        pending = controllers.call_async(ListControllers.Request())
                    if pending.done():
                        states = {c.name: c.state for c in pending.result().controller}
                        pending = None
                        next_check = now + 1
                        if states.get("joint_group_effort_controller") == "active":
                            launch(
                                "ros2",
                                "bag",
                                "record",
                                "--use-sim-time",
                                "-o",
                                str(data / "sensors"),
                                "--topics",
                                lidar,
                                "/imu/data",
                                "/odom",
                                "/tf",
                                "/tf_static",
                            )
                            recording = True
                            print(
                                "Go2 ready. Wait for recorder subscriptions, then start teleop.", flush=True
                            )
                if not recording and now - started > 180:
                    raise RuntimeError(f"capture readiness timeout; missing streams: {required - seen}")
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
    metadata = data / "sensors/metadata.yaml"
    counts = {}
    if metadata.exists():
        info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
        counts = {t["topic_metadata"]["name"]: t["message_count"] for t in info["topics_with_message_count"]}
    manifest["complete"] = samples > 0 and all(counts.get(t, 0) > 0 for t in required - {"truth"})
    manifest["counts"] = counts
    manifest["truth_samples"] = samples
    manifest_path.write_text(json.dumps(manifest, indent=2))
    if not manifest["complete"]:
        raise SystemExit("incomplete capture")


if __name__ == "__main__":
    main()
