"""Map A and localize B from its original ROS2 messages at recorded arrival speed."""

import argparse
import csv
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from competition import bagio, pcd
from competition.config import load, transform, pose7, write_json, phase_config
from competition import glim_reference, glim_route
from competition.go2_tracking import BodyOdometry, MapTracker
from competition.trajectory import save
from tools.go2_export import export
from tools.go2_global_seed import match
from tools.go2_live import build_map


def run(config, dest, glim_map=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    dest.mkdir(parents=True)
    bag = config["bags"]["localization"]
    if glim_map is None:
        build_map(config["bags"]["mapping"], dest)
        lidar_topic = "/utlidar/cloud_base"
        mount = None
    else:
        glim_reference.export(glim_map, dest)
        sensors = phase_config(config, "localization")
        lidar_topic = sensors["topics"]["lidar"]
        mount = transform(sensors["extrinsics"]["T_base_lidar"])
    tracker = MapTracker(pcd.xyz(dest / "map.pcd"), np.eye(4))
    odom = BodyOdometry()
    rclpy.init()
    node = rclpy.create_node("go2_map_tracker")
    pub = node.create_publisher(PoseStamped, "/competition/result/base_pose", 10)
    state_pub = node.create_publisher(String, "/competition/result/source", 10)
    latest_receipt = {}

    def observed(msg):
        latest_receipt[bagio.ns(msg)] = time.monotonic()

    node.create_subscription(PoseStamped, "/competition/result/base_pose", observed, 100)
    rows = []
    attempts = []
    status = []
    due = []
    odom_route = []
    first_stamp = first_arrival = search = locked = None
    checkpoints = iter([8.0, 16.0, 25.0, 40.0, 60.0, 90.0])
    checkpoint = next(checkpoints)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool, bagio.reader(bag) as r:
            conns = [c for c in r.connections if c.topic in (lidar_topic, "/utlidar/robot_odom")]
            for c, t, raw in r.messages(connections=conns):
                if first_arrival is None:
                    first_arrival = t
                    start = time.monotonic()
                arrival = start + (t - first_arrival) * 1e-9
                time.sleep(max(0.0, arrival - time.monotonic()))
                rclpy.spin_once(node, timeout_sec=0)
                m = r.deserialize(raw, c.msgtype)
                if c.topic.endswith("odom"):
                    odom.append(m)
                    continue
                stamp = bagio.ns(m)
                if first_stamp is None:
                    first_stamp = stamp
                sample = odom.at_scan(m, mount)
                if locked is None:
                    if search is not None and search.done():
                        result = search.result()
                        attempts.append(result)
                        search = None
                        if result["separated"]:
                            # prefix() returns the odometry frame of B's first scan.
                            tracker.map_from_odom = transform(
                                result["initial_pose"]
                            ) @ np.linalg.inv(transform(result["first_odometry_pose"]))
                            locked = stamp
                    if (
                        locked is None
                        and search is None
                        and (stamp - first_stamp) * 1e-9 >= checkpoint + 0.12
                    ):
                        search = pool.submit(
                            match,
                            dest / "map.pcd",
                            dest / "mapping.tum",
                            bag,
                            checkpoint,
                            lidar_topic,
                            mount,
                        )
                        checkpoint = next(checkpoints, float("inf"))
                if locked is None:
                    status.append((stamp, "uninitialized"))
                    continue
                if sample is None:
                    status.append((stamp, "missing_odometry"))
                    continue
                body, points = sample
                pose, source = tracker.update(body, points)
                values = pose7(pose)
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = values[:3]
                (
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ) = values[3:]
                pub.publish(msg)
                state = String()
                state.data = source
                state_pub.publish(state)
                rclpy.spin_once(node, timeout_sec=0)
                rows.append([stamp * 1e-9, *values])
                due.append((stamp, arrival))
                odom_route.append(body[:3, 3])
                status.append((stamp, source))
            until = time.monotonic() + 1
            while len(latest_receipt) < len(rows) and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    write_json(dest / "initialization.json", attempts)
    with (dest / "tracking.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(("stamp_ns", "source"))
        writer.writerows(status)
    if not rows:
        raise RuntimeError("No confirmed automatic localization on B")
    traj = np.asarray(rows)
    save(dest / "trajectory.tum", traj)
    latency = [(latest_receipt[n] - t) * 1000 for n, t in due if n in latest_receipt]
    deadlines = sum(
        latest_receipt.get(n, np.inf) > t_next for (n, _), (_, t_next) in zip(due, due[1:])
    )
    lengths = lambda xyz: float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
    counts = {s: sum(state == s for _, state in status) for s in sorted({s for _, s in status})}
    stats = dict(
        input_scans=len(status),
        sources=counts,
        output_poses=len(rows),
        observed_ros_poses=len(latency),
        first_pose_s=latest_receipt[due[0][0]] - start,
        max_gap_s=float(np.diff(traj[:, 0]).max()),
        tail_without_pose_s=(status[-1][0] - due[-1][0]) * 1e-9,
        length_m=lengths(traj[:, 1:4]),
        odometry_same_stamps_m=lengths(odom_route),
        arrival_to_pose_ms=dict(
            zip(("p50", "p95", "p99", "max"), np.percentile(latency, [50, 95, 99, 100]).tolist())
        ),
        missed_next_arrival=int(deadlines),
        deadline_comparisons=len(due) - 1,
    )
    write_json(dest / "timing.json", stats)
    if glim_map is not None:
        glim_route.export(dest, Path(config["output"] + "-B") / "mapping")
    export(dest)
    print(stats, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", default="/mgts/configs/competition/official-go2.yaml")
    parser.add_argument("--glim", action="store_true", help="Use the completed GLIM map for A")
    args = parser.parse_args()
    config = load(args.config)
    run(config, args.output, Path(config["output"]) / "mapping" if args.glim else None)
