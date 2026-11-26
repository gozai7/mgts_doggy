"""Benchmark NDT from the raw Go2 cloud topic at the recorded arrival rate."""

import argparse
import json
from pathlib import Path
import signal
import subprocess
from threading import Thread
import time

import numpy as np
from competition import backends, bagio, cloud
from competition.config import load, transform, pose7, write_json
from competition.trajectory import save
from tools.go2_global_seed import match


def build_map(bag, folder):
    """Aggregate only A's onboard deskewed returns, already expressed in A odom."""
    from competition.cold_start import sample

    points, poses = [], []
    with bagio.reader(bag) as r:
        channels = [
            c
            for c in r.connections
            if c.topic in ("/utlidar/cloud_deskewed", "/utlidar/robot_odom")
        ]
        for c, _, data in r.messages(connections=channels):
            m = r.deserialize(data, c.msgtype)
            if c.topic.endswith("odom"):
                poses.append([bagio.ns(m) * 1e-9, *bagio.pose_values(m.pose.pose)])
            else:
                pts = cloud.unpack(m)[0]
                points.append(pts[np.isfinite(pts).all(1) & np.any(pts != 0, axis=1)])
    xyz = sample(np.vstack(points), 0.065, 1000000).astype("<f4")
    header = f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n"
    (folder / "map.pcd").write_bytes(header.encode() + xyz.tobytes())
    save(folder / "mapping.tum", np.array(poses), csv=False)


def run(c, bag_a, bag_b, dest, unguarded=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from tf2_ros import TransformBroadcaster
    from diagnostic_msgs.msg import DiagnosticArray
    from rosgraph_msgs.msg import Clock

    dest.mkdir(parents=True)
    build_map(bag_a, dest)
    c["localization"]["initial_pose"] = None
    if c["localization"]["prediction"] != "constant_velocity":
        raise ValueError("This benchmark feeds the constant-velocity NDT tracker")
    params = dest / "ndt.yaml"
    settings = backends.localization_config(c, params, dest / "map.pcd")
    if not unguarded:
        settings.update(
            enable_map_odom_tf=True,
            use_odom_tf_prediction=True,
            predict_pose_from_previous_delta=False,
            enable_seed_correction_guard=True,
            seed_correction_guard_warmup_accepts=0,
            seed_correction_guard_release_rejections=100000,
            seed_correction_guard_translation_m=0.15,
            seed_correction_guard_yaw_deg=5.0,
            use_continuous_time_deskew=False,
        )
        backends.yaml_params(params, settings)
    rclpy.init()
    node = Node("go2_live_benchmark")
    odom_tf = TransformBroadcaster(node)
    pub = node.create_publisher(PointCloud2, "/go2/scan", 10)
    init = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    clock = node.create_publisher(Clock, "/clock", 10)
    poses = {}
    finished = {}
    status = {}
    sent = {}
    due = {}

    def on_pose(m):
        n = bagio.ns(m)
        finished[n] = time.monotonic()
        poses[n] = bagio.pose_values(m.pose.pose)

    def on_status(m):
        for s in m.status:
            if s.name.endswith("/alignment"):
                status[bagio.ns(m)] = (s.message, {v.key: v.value for v in s.values})

    node.create_subscription(PoseWithCovarianceStamped, "/pcl_pose", on_pose, 200)
    node.create_subscription(DiagnosticArray, "/alignment_status", on_status, 200)
    spin = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    def set_clock(stamp):
        m = Clock()
        m.clock.sec, m.clock.nanosec = divmod(stamp, 10**9)
        clock.publish(m)

    proc = subprocess.Popen(
        [
            "ros2",
            "run",
            "lidar_localization_ros2",
            "lidar_localization_node",
            "--ros-args",
            "--params-file",
            str(params),
            "-r",
            "cloud:=/go2/scan",
        ],
        stdout=(dest / "algorithm.log").open("w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        for transition in ["configure", "activate"]:
            subprocess.run(
                ["ros2", "lifecycle", "set", "/lidar_localization", transition],
                check=True,
                timeout=45,
            )
        deadline = time.monotonic() + 15
        while not pub.get_subscription_count() or not init.get_subscription_count():
            if time.monotonic() > deadline:
                raise RuntimeError("ROS subscriptions unavailable")
            time.sleep(0.05)
        with bagio.reader(bag_b) as reader:
            channels = [
                c
                for c in reader.connections
                if c.topic in ("/utlidar/cloud_base", "/utlidar/robot_odom")
            ]
            first_arrival = None
            initial_odom = None
            latest_odom = None
            first_scan = None
            search = None
            active = False
            scans = 0
            prestart = 0
            # Each arrival has a fixed deadline. The publisher never waits for an NDT result.
            for conn, arrival, raw in reader.messages(connections=channels):
                if first_arrival is None:
                    first_arrival = arrival
                    start = time.monotonic()
                release = start + (arrival - first_arrival) * 1e-9
                time.sleep(max(0.0, release - time.monotonic()))
                if conn.topic.endswith("odom"):
                    od = reader.deserialize(raw, conn.msgtype)
                    latest_odom = transform(bagio.pose_values(od.pose.pose))
                    if not unguarded:
                        tf = TransformStamped()
                        tf.header.frame_id = od.header.frame_id
                        tf.child_frame_id = od.child_frame_id
                        tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(bagio.ns(od), 10**9)
                        pos = od.pose.pose.position
                        q = od.pose.pose.orientation
                        (
                            tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z,
                        ) = pos.x, pos.y, pos.z
                        (
                            tf.transform.rotation.x,
                            tf.transform.rotation.y,
                            tf.transform.rotation.z,
                            tf.transform.rotation.w,
                        ) = q.x, q.y, q.z, q.w
                        odom_tf.sendTransform(tf)
                    if initial_odom is None:
                        initial_odom = latest_odom
                    continue
                m = reader.deserialize(raw, conn.msgtype)
                n = bagio.ns(m)
                if first_scan is None:
                    first_scan = n
                scans += 1
                set_clock(n)
                if search is None and (n - first_scan) * 1e-9 >= 25.1:
                    # Replay has received the whole 25 s prefix used by the estimator.
                    search = match(dest / "map.pcd", dest / "mapping.tum", bag_b, 25.0)
                    write_json(dest / "initialization.json", search)
                    if not search["separated"]:
                        raise RuntimeError("Ambiguous automatic start")
                if search is None or (not active and release < time.monotonic() - 0.05):
                    prestart += 1
                    continue
                if not active:
                    if initial_odom is None:
                        raise ValueError("Missing relative motion during initialization")
                    prior = (
                        transform(search["initial_pose"])
                        @ np.linalg.inv(initial_odom)
                        @ latest_odom
                    )
                    msg = PoseWithCovarianceStamped()
                    msg.header.frame_id = c["frames"]["map"]
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(n, 10**9)
                    xyzq = pose7(prior)
                    p = msg.pose.pose
                    p.position.x, p.position.y, p.position.z = xyzq[:3]
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = xyzq[3:]
                    init.publish(msg)
                    active = True
                    time.sleep(0.025)
                # cloud_base has already been transformed to the body frame on the robot.
                normalized = cloud.normalize(m, c)
                normalized.header.frame_id = c["frames"]["base"]
                ros_msg = deserialize_message(
                    bytes(bagio.STORE.serialize_cdr(normalized, normalized.__msgtype__)),
                    PointCloud2,
                )
                due[n] = release
                sent[n] = time.monotonic()
                pub.publish(ros_msg)
        end = time.monotonic() + 3
        while len(status) < len(sent) and time.monotonic() < end:
            time.sleep(0.01)
        time.sleep(0.10)
        ids = list(sent)
        ok = [
            n
            for n in ids
            if n in poses
            and status.get(n, ("",))[0]
            in ("ok", "alternate_seed_recovered", "recovery_retry_from_last_pose_recovered")
        ]
        if not ok:
            raise RuntimeError("No accepted matching poses")

        def pct(a):
            return dict(
                zip(("p50", "p95", "p99", "max"), np.percentile(a, [50, 95, 99, 100]).tolist())
            )

        accepted_stamps = set(ok)
        late = sum(n not in accepted_stamps or finished[n] > due[j] for n, j in zip(ids, ids[1:]))
        report = {
            "predictor": "constant_velocity" if unguarded else "kinematic_tf_guarded",
            "scans_in_b": scans,
            "before_lock": prestart,
            "sent": len(ids),
            "status": len(status),
            "accepted": len(ok),
            "missed_next_arrival_or_no_pose": late,
            "deadline_comparisons": len(ids) - 1,
            "first_pose_wall_s": min(finished[n] for n in ok) - start,
            "arrival_to_pose_ms": pct([(finished[n] - due[n]) * 1000 for n in ok]),
            "publish_to_pose_ms": pct([(finished[n] - sent[n]) * 1000 for n in ok]),
            "ndt_alignment_ms": pct([float(status[n][1]["alignment_time_sec"]) * 1000 for n in ok]),
            "pose_span_s": (ok[-1] - ok[0]) * 1e-9,
        }
        report["path_length_m"] = float(
            np.linalg.norm(np.diff([poses[n][:3] for n in ok], axis=0), axis=1).sum()
        )
        save(dest / "trajectory.tum", np.array([[n * 1e-9, *poses[n]] for n in ok]))
        write_json(dest / "timing.json", report)
        print(report, flush=True)
    finally:
        import os

        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        node.destroy_node()
        rclpy.shutdown()
        spin.join(2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--config", default="/mgts/configs/competition/official-go2.yaml")
    parser.add_argument(
        "--unguarded", action="store_true", help="Reproduce the constant velocity comparison"
    )
    args = parser.parse_args()
    config = load(args.config)
    run(
        config,
        config["bags"]["mapping"],
        config["bags"]["localization"],
        args.output,
        args.unguarded,
    )
    from tools.go2_export import export

    export(args.output)
