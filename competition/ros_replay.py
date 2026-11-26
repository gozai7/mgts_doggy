"""ROS-only lockstep replay: wait for a real registration result for each scan."""
import argparse
import collections
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from . import bagio, cloud
from .config import load, transform, write_json
from .trajectory import save, change_body


def run(config, normalized, output, params, baseline=False):
    # These imports intentionally stay out of the ROS-free utility modules.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from ros2run.api import get_executable_path
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from rosgraph_msgs.msg import Clock
    from lifecycle_msgs.srv import ChangeState, GetState
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    c = load(config); output = Path(output); output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("competition_replay")
    reliable = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {"/competition/lidar": node.create_publisher(PointCloud2, "/competition/lidar", reliable),
            "/competition/imu": node.create_publisher(Imu, "/competition/imu", reliable),
            "/competition/odometry": node.create_publisher(Odometry, "/competition/odometry", reliable)}
    clock_pub = node.create_publisher(Clock, "/clock", reliable)
    static, dynamic = StaticTransformBroadcaster(node), TransformBroadcaster(node)
    samples, statuses, events, initialized = {}, {}, [], [not c["localization"].get("manual_initial_pose", False)]
    diagnostic_stream = (output / "alignment_status.jsonl").open("w")

    def stamp(m):
        return int(m.header.stamp.sec) * 1000000000 + int(m.header.stamp.nanosec)

    def pose_callback(m):
        p = m.pose.pose
        samples[stamp(m)] = [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]

    def diagnostic(m):
        for s in m.status:
            if s.name == "lidar_localization_ros2/alignment":
                n = stamp(m)
                values = {x.key: x.value for x in s.values}
                statuses[n] = {"stamp_ns": n, "level": ord(s.level), "message": s.message, "values": values}
                diagnostic_stream.write(json.dumps(statuses[n]) + "\n"); diagnostic_stream.flush()

    subs = [node.create_subscription(Odometry if baseline else PoseWithCovarianceStamped,
            "/kiss/odometry" if baseline else "/pcl_pose", pose_callback, reliable if baseline else latched),
            node.create_subscription(DiagnosticArray, "/alignment_status", diagnostic, latched),
            node.create_subscription(PoseWithCovarianceStamped, "/initialpose", lambda _: initialized.__setitem__(0, True), reliable)]
    # Only small IMU/odometry records are retained. Clouds stream from disk.
    sensors = []
    with bagio.reader(normalized) as r:
        start = r.start_time
        if not baseline:
            for conn, n, raw in r.messages(connections=[x for x in r.connections if x.topic in ("/competition/imu", "/competition/odometry")]):
                typ = Imu if conn.topic.endswith("imu") else Odometry
                sensors.append((n, conn.topic, deserialize_message(bytes(raw), typ)))
    current_clock = start

    def clock(n):
        nonlocal current_clock
        current_clock = max(current_clock, int(n))
        msg = Clock(); msg.clock.sec, msg.clock.nanosec = divmod(current_clock, 1000000000)
        clock_pub.publish(msg)

    tfs = []
    for sensor in ("lidar", "imu"):
        values = c["extrinsics"].get("T_base_" + sensor)
        if values is None:
            continue
        t = TransformStamped(); t.header.frame_id = c["frames"]["base"]; t.child_frame_id = c["frames"][sensor]
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = map(float, values[:3])
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w = map(float, values[3:])
        tfs.append(t)
    static.sendTransform(tfs); clock(start)
    if baseline:
        package, executable = "kiss_icp", "kiss_icp_node"
        remaps = ["-r", "pointcloud_topic:=/competition/lidar"]
    else:
        package, executable = "lidar_localization_ros2", "lidar_localization_node"
        remaps = ["-r", "__node:=lidar_localization", "-r", "cloud:=/competition/lidar",
                  "-r", "imu:=/competition/imu", "-r", "odom:=/competition/odometry"]
    # Start the ROS node itself so replay can stop and reap it between runs.
    path = get_executable_path(package_name=package, executable_name=executable)
    if path is None:
        raise RuntimeError(f"Executable {package}/{executable} not found")
    cmd = [path, "--ros-args", "--params-file", str(params), *remaps]
    log = (output / "algorithm.log").open("w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)

    def wait_for(predicate, timeout, description):
        until = time.monotonic() + timeout
        while not predicate():
            if proc.poll() is not None:
                raise RuntimeError(f"Algorithm exited {proc.returncode}; see {output / 'algorithm.log'}")
            if time.monotonic() > until:
                raise TimeoutError(description)
            clock(current_clock)
            rclpy.spin_once(node, timeout_sec=.01)

    def lifecycle_transition(transition_id):
        client = node.create_client(ChangeState, "/lidar_localization/change_state")
        wait_for(client.service_is_ready, 30, "Lifecycle change_state service unavailable")
        req = ChangeState.Request(); req.transition.id = transition_id
        future = client.call_async(req)
        wait_for(future.done, c["localization"]["initialization_timeout_s"], "Map loading/lifecycle timed out")
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f"Lifecycle transition {transition_id} failed")
        node.destroy_client(client)

    try:
        if not baseline:
            lifecycle_transition(1)  # configure (load map)
            lifecycle_transition(3)  # activate
        wait_for(lambda: pubs["/competition/lidar"].get_subscription_count() > 0, 30, "LiDAR subscriber unavailable")
        if not baseline and not initialized[0]:
            print("Set BASE initial pose in RViz using /initialpose; waiting before consuming clouds", flush=True)
            wait_for(lambda: initialized[0], c["localization"]["initialization_timeout_s"], "No manual initial pose received")
        # Discovery and static TF propagation, not a readiness substitute.
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            clock(start); rclpy.spin_once(node, timeout_sec=.01)
        index = 0
        with bagio.reader(normalized) as r:
            for conn, _, raw in r.messages(connections=[x for x in r.connections if x.topic == "/competition/lidar"]):
                typed = r.deserialize(raw, conn.msgtype)
                n = bagio.ns(typed)
                arr = cloud.fields(typed)
                lookahead = max(0., float(arr["time"].max())) if "time" in arr.dtype.names and len(arr) else 0.
                ready_at = n + round(lookahead * 1e9)
                # Supply IMU covering the sweep. The bag's IMU history remains in chronological order.
                while index < len(sensors) and sensors[index][0] <= ready_at:
                    t, topic, m = sensors[index]; clock(t); pubs[topic].publish(m)
                    if topic.endswith("odometry") and c["localization"]["prediction"] == "kinematic":
                        tf = TransformStamped(); tf.header = m.header; tf.child_frame_id = m.child_frame_id
                        tf.transform.translation.x = m.pose.pose.position.x
                        tf.transform.translation.y = m.pose.pose.position.y
                        tf.transform.translation.z = m.pose.pose.position.z
                        tf.transform.rotation = m.pose.pose.orientation; dynamic.sendTransform(tf)
                    index += 1
                    rclpy.spin_once(node, timeout_sec=0)
                clock(ready_at)
                # Allow separate IMU callback group to consume the buffered sweep before alignment.
                until = time.monotonic() + .02
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.001)
                pubs["/competition/lidar"].publish(deserialize_message(bytes(raw), PointCloud2))
                wait_for(lambda: n in (samples if baseline else statuses), c["localization"]["scan_timeout_s"],
                         f"No registration result for scan {n}; refusing silent scan loss")
                if not baseline:
                    s = statuses[n]
                    accepted = s["message"] in ("ok", "alternate_seed_recovered", "recovery_retry_from_last_pose_recovered")
                    if accepted:
                        wait_for(lambda: n in samples, 5, "Accepted registration has no matching pose")
                    values = s["values"]
                    events.append({"t": n * 1e-9, "stamp_ns": n, "state": "relocalized_locally" if accepted and s["message"] != "ok" else "localized" if accepted else
                        "relocalization_requested" if values.get("reinitialization_requested") == "true" else "registration_rejected",
                        "upstream_state": values.get("recovery_state"), "message": s["message"],
                        "accepted": accepted, "fitness_score": values.get("fitness_score")})
                else:
                    events.append({"t": n * 1e-9, "stamp_ns": n, "state": "lidar_odometry", "accepted": True})
                if len(events) % 100 == 0:
                    print(f"Processed {len(events)} scans", flush=True)
        accepted_times = {x["stamp_ns"] for x in events if x["accepted"]}
        rows = [[n * 1e-9] + p for n, p in sorted(samples.items()) if n in accepted_times]
        if len(rows) < 2:
            raise RuntimeError("Fewer than two accepted poses; localization unsuccessful")
        a = np.asarray(rows)
        if baseline:
            a = change_body(a, np.linalg.inv(transform(c["extrinsics"]["T_base_lidar"])))
        save(output / "trajectory.tum", a)
        accepted = sum(e["accepted"] for e in events)
        streak = worst = 0; lost_time = 0.
        for i, e in enumerate(events):
            streak = 0 if e["accepted"] else streak + 1; worst = max(worst, streak)
            if i and not events[i-1]["accepted"]:
                lost_time += e["t"] - events[i-1]["t"]
        report = {"scans": len(events), "accepted": accepted, "accepted_fraction": accepted / len(events),
                  "maximum_rejection_streak": worst, "time_without_accepted_registration_s": lost_time,
                  "odometry_only_time_s": 0., "automatic_global_relocalization": False,
                  "local_recoveries": sum(e["state"] == "relocalized_locally" for e in events),
                  "global_relocalizations": 0, "events": events}
        write_json(output / "diagnostics.json", report)
        if not baseline and report["accepted_fraction"] < c["localization"]["min_accepted_fraction"]:
            raise RuntimeError("Accepted fraction below configured minimum; outputs retained as failed run")
    finally:
        proc.send_signal(signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill(); proc.wait()
        diagnostic_stream.close(); log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--config", required=True); p.add_argument("--bag", required=True)
    p.add_argument("--output", required=True); p.add_argument("--params", required=True); p.add_argument("--baseline", action="store_true")
    a = p.parse_args(); run(a.config, a.bag, a.output, a.params, a.baseline)
