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
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from geometry_msgs.msg import PoseArray
    from diagnostic_msgs.msg import DiagnosticArray
    from rosgraph_msgs.msg import Clock
    from lifecycle_msgs.srv import ChangeState, GetState
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from std_srvs.srv import Trigger
    c = load(config); output = Path(output); output.mkdir(parents=True, exist_ok=True)
    beluga = not baseline and c["localization"].get("backend") == "beluga"
    namespaced_ndt = not baseline and not beluga and "profile" in c["localization"]
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
    pose_stream = (output / "poses.jsonl").open("w")
    event_stream = (output / "scan_events.jsonl").open("w")
    active_scan = None
    particle_stream = (output / "particles.jsonl").open("w") if beluga else None
    particle_updates = 0

    def particle_callback(m):
        nonlocal particle_updates
        particle_updates += 1
        if particle_updates % c.get("beluga",{}).get("particle_export_every_n",10) == 1:
            particle_stream.write(json.dumps({"stamp_ns":stamp(m),"frame":m.header.frame_id,
                "poses":[[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w] for p in m.poses]})+"\n")
            particle_stream.flush()

    def stamp(m):
        return int(m.header.stamp.sec) * 1000000000 + int(m.header.stamp.nanosec)

    def pose_callback(m):
        p = m.pose.pose
        samples[stamp(m)] = [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if stamp(m)==active_scan:
            values=samples[stamp(m)]
            if baseline:
                from .config import pose7
                values=pose7(transform(values)@np.linalg.inv(transform(c["extrinsics"]["T_base_lidar"])))
            pose_stream.write(json.dumps({"stamp_ns":stamp(m),"pose":values})+"\n"); pose_stream.flush()

    def persist_event():
        events[-1]["wall_elapsed_s"]=time.monotonic()-replay_started
        event_stream.write(json.dumps(events[-1])+"\n"); event_stream.flush()

    def diagnostic(m):
        for s in m.status:
            if s.name == ("mgts_beluga/alignment" if beluga else "lidar_localization_ros2/alignment"):
                n = stamp(m)
                values = {x.key: x.value for x in s.values}
                statuses[n] = {"stamp_ns": n, "level": s.level if isinstance(s.level, int) else ord(s.level), "message": s.message, "values": values}
                diagnostic_stream.write(json.dumps(statuses[n]) + "\n"); diagnostic_stream.flush()

    subs = [node.create_subscription(Odometry if baseline else PoseWithCovarianceStamped,
            "/kiss/odometry" if baseline else "/beluga/pose" if beluga else "/ndt_gicp/pose" if namespaced_ndt else "/pcl_pose", pose_callback, reliable if baseline else latched),
            node.create_subscription(DiagnosticArray, "/beluga/alignment_status" if beluga else "/ndt_gicp/alignment_status" if namespaced_ndt else "/alignment_status", diagnostic, latched),
            node.create_subscription(PoseWithCovarianceStamped, "/initialpose", lambda _: initialized.__setitem__(0, True), reliable)]
    if beluga:
        subs.append(node.create_subscription(PoseArray,"/beluga/particles",particle_callback,
                    QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)))
    # Only small IMU/odometry records are retained. Clouds stream from disk.
    sensors = []
    external = c.get("odometry", {}).get("trajectory_jsonl") if not baseline else None
    external_poses = {}
    with bagio.reader(normalized) as r:
        start = r.start_time
        if not baseline:
            sensor_connections=[x for x in r.connections if x.topic in ("/competition/imu", "/competition/odometry")]
            # AnyReader treats an empty connection list as "all topics".
            for conn, n, raw in (r.messages(connections=sensor_connections) if sensor_connections else ()):
                if external and conn.topic.endswith("odometry"):
                    continue
                typ = Imu if conn.topic.endswith("imu") else Odometry
                sensors.append((n, conn.topic, deserialize_message(bytes(raw), typ)))
    if external:
        for line in Path(external).read_text().splitlines():
            row = json.loads(line); n, p = row["stamp_ns"], row["pose"]
            external_poses[n] = p
            m = Odometry(); m.header.frame_id = c["frames"]["odom"]; m.child_frame_id = c["frames"]["base"]
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(n, 10**9)
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p[:3]
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = p[3:]
            variance = c["odometry"].get("position_variance_m2", .1)
            angle_variance = c["odometry"].get("orientation_variance_rad2", .05)
            if min(variance, angle_variance) <= 0:
                raise ValueError("External odometry uncertainty must be positive")
            for k in (0,7,14): m.pose.covariance[k] = variance
            for k in (21,28,35): m.pose.covariance[k] = angle_variance
            sensors.append((n, "/competition/odometry", m))
        sensors.sort(key=lambda record: record[0])
    odometry_array = np.array([[n*1e-9,*p] for n,p in sorted(external_poses.items())]) if external_poses else None
    if odometry_array is not None:
        # IEEE double epoch seconds can coalesce stamps less than one ULP apart; retain integer stamps for ROS IO.
        _, unique = np.unique(odometry_array[:,0],return_index=True)
        odometry_array=odometry_array[np.sort(unique)]
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
    from .localization import Guard
    from .config import pose7
    live_guard = Guard(c.get("recovery"))
    last_reinit = None
    reinit_client = node.create_client(Trigger, "/beluga/global_localization") if beluga else None
    if baseline:
        cmd = ["ros2","run","kiss_icp","kiss_icp_node","--ros-args","--params-file",str(params),"-r","pointcloud_topic:=/competition/lidar"]
    elif beluga:
        cmd = ["ros2", "run", "mgts_localization", "beluga_node", "--ros-args", "--params-file", str(params)]
    else:
        cmd = ["ros2","run","lidar_localization_ros2","lidar_localization_node","--ros-args",
               "-r","__node:=lidar_localization","--params-file",str(params),"-r","cloud:=/competition/lidar",
               "-r","imu:=/competition/imu","-r","odom:=/competition/odometry"]
        if namespaced_ndt:
            cmd.extend(["-r","pcl_pose:=/ndt_gicp/pose","-r","alignment_status:=/ndt_gicp/alignment_status"])
    log = (output / "algorithm.log").open("w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    replay_started=time.monotonic()

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
        if not baseline and not beluga:
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
                    if topic.endswith("odometry") and c["localization"]["prediction"] in ("kinematic", "external_tf"):
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
                if external and n not in external_poses:
                    events.append({"t": n*1e-9,"stamp_ns":n,"state":"missing_odometry","accepted":False})
                    live_guard.update(n,None,None,False)
                    persist_event()
                    continue
                # A latched initial pose can share the first scan's stamp; require fresh alignment output.
                samples.pop(n,None); statuses.pop(n,None)
                scan_message=deserialize_message(bytes(raw),PointCloud2)
                deskew_counts={}
                if beluga and c["point_time"]["mode"]!="none":
                    from .trajectory import deskew_scan
                    xyz,fields=cloud.unpack(typed)
                    corrected,valid=deskew_scan(xyz,fields["time"],n*1e-9,odometry_array,transform(c["extrinsics"]["T_base_lidar"]),c["odometry"].get("max_gap_s",.5))
                    deskew_counts={"deskewed_points":int(valid.sum()),"points_without_motion":int((~valid).sum())}
                    if len(corrected)<5:
                        events.append({"t":n*1e-9,"stamp_ns":n,"state":"missing_motion_for_deskew","accepted":False,**deskew_counts})
                        live_guard.update(n,None,None,False)
                        persist_event()
                        continue
                    adjusted=cloud.pack(corrected,n,c["frames"]["lidar"])
                    scan_message=deserialize_message(bytes(bagio.STORE.serialize_cdr(adjusted,adjusted.__msgtype__)),PointCloud2)
                active_scan=n
                pubs["/competition/lidar"].publish(scan_message)
                wait_for(lambda: n in (samples if baseline else statuses), c["localization"]["scan_timeout_s"],
                         f"No registration result for scan {n}; refusing silent scan loss")
                if not baseline:
                    s = statuses[n]
                    accepted = s["message"] in ("ok", "alternate_seed_recovered", "recovery_retry_from_last_pose_recovered")
                    if accepted or beluga:
                        wait_for(lambda: n in samples, 5, "Accepted registration has no matching pose")
                    values = s["values"]
                    events.append({"t": n * 1e-9, "stamp_ns": n, "state": "relocalized_locally" if accepted and s["message"] != "ok" else "localized" if accepted else
                        "relocalization_requested" if values.get("reinitialization_requested") == "true" else "registration_rejected",
                        "upstream_state": values.get("recovery_state"), "message": s["message"],
                        "accepted": accepted, "fitness_score": values.get("fitness_score"), "values": values,**deskew_counts})
                    if external:
                        state, source, guarded = live_guard.update(n,samples.get(n),external_poses[n],accepted)
                        events[-1].update(supervisor_state=state,pose_source=source)
                        if guarded is not None:
                            anchor = pose7(transform(guarded) @ np.linalg.inv(transform(external_poses[n])))
                            tf = TransformStamped(); tf.header.stamp = typed.header.stamp
                            tf.header.frame_id = c["frames"]["map"]; tf.child_frame_id = c["frames"]["odom"]
                            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=anchor[:3]
                            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=anchor[3:]
                            dynamic.sendTransform(tf)
                        cooldown=c.get("recovery",{}).get("reinitialization_cooldown_s",15.)
                        wrong_start = beluga and state=="GLOBAL_SEARCH" and c["localization"]["initial_pose"] is not None and (n-start)*1e-9>c.get("recovery",{}).get("global_search_timeout_s",15.)
                        if beluga and (state=="LOST" or wrong_start) and (last_reinit is None or (n-last_reinit)*1e-9>=cooldown):
                            if reinit_client.service_is_ready():
                                future=reinit_client.call_async(Trigger.Request())
                                wait_for(future.done,10,"Beluga global reset timed out")
                                if not future.result().success: raise RuntimeError("Beluga global reset failed")
                                last_reinit=n; events[-1]["global_reset_requested"]=True
                else:
                    events.append({"t": n * 1e-9, "stamp_ns": n, "state": "lidar_odometry", "accepted": True})
                if len(events) % 100 == 0:
                    print(f"Processed {len(events)} scans", flush=True)
                persist_event()
        accepted_times = {x["stamp_ns"] for x in events if x["accepted"] or beluga}
        rows = [[n * 1e-9] + p for n, p in sorted(samples.items()) if n in accepted_times]
        if len(rows) < 2 and not c["localization"].get("allow_failed_run_export", False):
            raise RuntimeError("Fewer than two accepted poses; localization unsuccessful")
        a = np.asarray(rows).reshape(-1,8)
        if baseline:
            a = change_body(a, np.linalg.inv(transform(c["extrinsics"]["T_base_lidar"])))
        pose_stream.close()
        with (output / "poses.jsonl").open("w") as stream:
            for n, row in zip(sorted(n for n in samples if n in accepted_times), a):
                stream.write(json.dumps({"stamp_ns": n, "pose": row[1:].tolist()}) + "\n")
        save(output / "trajectory.tum", a)
        accepted = sum(e["accepted"] for e in events)
        streak = worst = 0; lost_time = 0.
        for i, e in enumerate(events):
            streak = 0 if e["accepted"] else streak + 1; worst = max(worst, streak)
            if i and not events[i-1]["accepted"]:
                lost_time += e["t"] - events[i-1]["t"]
        report = {"scans": len(events), "accepted": accepted, "accepted_fraction": accepted / len(events) if events else 0.,
                  "maximum_rejection_streak": worst, "time_without_accepted_registration_s": lost_time,
                  "odometry_only_time_s": 0., "automatic_global_relocalization": beluga,
                  "local_recoveries": sum(e["state"] == "relocalized_locally" for e in events),
                  "global_relocalizations": 0, "events": events}
        report["global_reinitialization_requests"]=sum(bool(e.get("global_reset_requested")) for e in events)
        pending_reset=False
        for event in events:
            if event.get("global_reset_requested"): pending_reset=True
            elif pending_reset and event.get("supervisor_state")=="TRACKING":
                report["global_relocalizations"]+=1; pending_reset=False
        write_json(output / "diagnostics.json", report)
        if not baseline and not beluga and not c["localization"].get("allow_failed_run_export", False) and report["accepted_fraction"] < c["localization"]["min_accepted_fraction"]:
            raise RuntimeError("Accepted fraction below configured minimum; outputs retained as failed run")
    finally:
        proc.send_signal(signal.SIGINT) if proc.poll() is None else None
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill(); proc.wait()
        diagnostic_stream.close()
        pose_stream.close(); event_stream.close()
        if particle_stream is not None: particle_stream.close()
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--config", required=True); p.add_argument("--bag", required=True)
    p.add_argument("--output", required=True); p.add_argument("--params", required=True); p.add_argument("--baseline", action="store_true")
    a = p.parse_args(); run(a.config, a.bag, a.output, a.params, a.baseline)
