"""Process a mapping or inspection bag in an isolated ROS graph."""

import argparse
import json
import math
import signal
import subprocess
import time
from contextlib import ExitStack
from itertools import chain
from pathlib import Path
from shutil import copyfile

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from dataset import Dataset, rotation, stamp
from geometry import compose, csv_save, inverse, load_amcl_map
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile
from refine import refine
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def transform(stamp_msg, parent, child, pose):
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp_msg, parent, child
    tf.transform.translation.x, tf.transform.translation.y = map(float, pose[:2])
    tf.transform.rotation.z = math.sin(pose[2] / 2)
    tf.transform.rotation.w = math.cos(pose[2] / 2)
    return tf


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def save_map(grid, folder):
    h, w = grid.info.height, grid.info.width
    cells = np.array(grid.data).reshape(h, w)[::-1]
    image = np.select([cells < 0, cells >= 65], [205, 0], default=254).astype("uint8")
    Image.fromarray(image).save(folder / "map.pgm")
    origin = grid.info.origin.position
    (folder / "map.yaml").write_text(
        yaml.safe_dump(
            dict(
                image="map.pgm",
                mode="trinary",
                resolution=grid.info.resolution,
                origin=[origin.x, origin.y, 0.0],
                negate=0,
                occupied_thresh=0.65,
                free_thresh=0.196,
            )
        )
    )


def run(args, out, dataset, first, remaining):
    node = rclpy.create_node("offline_solver")
    clock = node.create_publisher(Clock, "/clock", 10)
    laser = node.create_publisher(LaserScan, "/scan", 10)
    tf, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps, updates, odom_rows = [], {}, []
    mapping = args.mode == "map"
    amcl_received_map = False

    def on_amcl_log(msg):
        nonlocal amcl_received_map
        # Nav2 Jazzy logs map acceptance under the same mutex as global initialization.
        if msg.name == "amcl" and msg.msg.startswith("Received a ") and " map @ " in msg.msg:
            amcl_received_map = True

    def on_map(msg):
        maps[:] = [msg]

    def on_pose(msg):
        p = msg.pose.pose
        f = rotation(p.orientation).as_matrix()[:, 0]
        updates[stamp(msg.header)] = [p.position.x, p.position.y, math.atan2(f[1], f[0])]

    node.create_subscription(OccupancyGrid, "/map", on_map, durable)
    if not mapping:
        rosout_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Log, "/rosout", on_amcl_log, rosout_qos)
    node.create_subscription(
        PoseWithCovarianceStamped, "/pose" if mapping else "/amcl_pose", on_pose, 20 if mapping else durable
    )
    processes = []

    def spin_until(condition, seconds=20):
        end = time.monotonic() + seconds
        while not condition():
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("solver process exited; see ros.log")
            if time.monotonic() >= end:
                return False
            rclpy.spin_once(node, timeout_sec=0.01)
        return True

    def service(name, kind, req):
        client = node.create_client(kind, name)
        if not spin_until(client.service_is_ready):
            raise TimeoutError(name)
        future = client.call_async(req)
        if not spin_until(future.done, 60):
            raise TimeoutError(name)
        response = future.result()
        node.destroy_client(client)
        return response

    modes = []
    with ExitStack() as stack:
        log = stack.enter_context((out / "ros.log").open("w"))

        def start(package, binary, name, params):
            exe = Path(get_package_prefix(package), "lib", package, binary)
            process = subprocess.Popen(
                [str(exe), "--ros-args", "-r", f"__node:={name}", *params],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            processes.append(process)
            stack.callback(stop, process)
            for transition in [1, 3]:  # configure, activate
                req = ChangeState.Request()
                req.transition.id = transition
                if not service(f"/{name}/change_state", ChangeState, req).success:
                    raise RuntimeError(f"{name} lifecycle transition {transition} failed; see ros.log")

        clock.publish(Clock(clock=first[0].header.stamp))
        if mapping:
            start(
                "slam_toolbox",
                "async_slam_toolbox_node",
                "slam_toolbox",
                ["--params-file", "/mgts/configs/mapping.yaml"],
            )
        else:
            start(
                "nav2_map_server",
                "map_server",
                "map_server",
                ["-p", f"yaml_filename:={args.map.resolve()}", "-p", "use_sim_time:=true"],
            )
            params = ["--params-file", "/mgts/configs/amcl.yaml"]
            if args.initial is not None:
                params += ["-p", "set_initial_pose:=true"]
                for key, val in zip(["x", "y", "yaw"], args.initial):
                    params += ["-p", f"initial_pose.{key}:={val}"]
            start("nav2_amcl", "amcl", "amcl", params)
            if not spin_until(lambda: amcl_received_map):
                raise RuntimeError("AMCL did not acknowledge the reference map; see ros.log")
        if not spin_until(lambda: laser.get_subscription_count() and (mapping or maps)):
            raise RuntimeError("missing /scan subscriber or reference map")
        when = first[0].header.stamp
        static.sendTransform(transform(when, "base_footprint", "laser", [*dataset.offset, 0.0]))
        if not mapping and args.initial is None:
            if 0 not in maps[-1].data:
                raise ValueError("global localization requires a map with free cells")
            service("/reinitialize_global_localization", Empty, Empty.Request())

        previous = stamp(first[0].header)
        preview, refinement_scans = [], []
        for scan, odom in chain([first], remaining):
            t = stamp(scan.header)
            due = time.monotonic() + min(t - previous, 1.0) / args.rate
            spin_until(lambda: time.monotonic() >= due, max(2.0, 2.0 / args.rate))
            clock.publish(Clock(clock=scan.header.stamp))
            tf.sendTransform(transform(scan.header.stamp, "odom", "base_footprint", odom))
            laser.publish(scan)
            if not odom_rows:
                spin_until(lambda: bool(updates), 2.0)
            odom_rows.append([t, *odom])
            if args.refine:
                ranges = np.asarray(scan.ranges)
                indices = np.flatnonzero(np.isfinite(ranges))
                indices = indices[::max(1, math.ceil(len(indices) / 160))]
                angles = scan.angle_min + indices * scan.angle_increment
                points = np.column_stack((np.cos(angles), np.sin(angles))) * ranges[indices, None]
                refinement_scans.append(points + dataset.offset)
            if not mapping and len(odom_rows) % 5 == 1:
                rs = np.asarray(scan.ranges)[::6]
                angles = scan.angle_min + np.arange(len(rs)) * 6 * scan.angle_increment
                valid = np.isfinite(rs)
                hits = np.column_stack((rs[valid] * np.cos(angles[valid]), rs[valid] * np.sin(angles[valid])))
                preview.append(np.column_stack((np.full(len(hits), t), hits + dataset.offset)))
            previous = t
        if mapping:
            if not spin_until(lambda: maps and stamp(maps[-1].header) >= previous - 1e-6):
                raise RuntimeError("final map did not reach last scan; see ros.log")
            req = SerializePoseGraph.Request(filename=str(out.resolve() / "map"))
            if service("/slam_toolbox/serialize_map", SerializePoseGraph, req).result:
                raise RuntimeError("SLAM graph export failed")
            save_map(maps[-1], out)
            subprocess.run(
                ["export_graph", out / "map.posegraph", out / "trajectory.csv"], check=True, stdout=log
            )
        else:
            end = time.monotonic() + 0.5
            spin_until(lambda: time.monotonic() >= end)
            if not updates:
                raise RuntimeError("AMCL produced no poses; see ros.log")
            # Keep the latest scan correction and propagate local odometry between updates.
            correction = None
            poses = []
            for t, *od in odom_rows:
                if t in updates:
                    correction = compose(updates[t], inverse(od))
                if correction is not None:
                    poses.append([t, *compose(correction, od)])
            if not poses:
                raise RuntimeError("no AMCL estimate matches a scan timestamp")
            if args.refine:
                csv_save(out / "amcl.csv", poses)
                poses, modes = refine(maps[-1], odom_rows, poses, refinement_scans, args.initial)
                print("Refinement segments:", ", ".join(modes), flush=True)
            csv_save(out / "trajectory.csv", poses)
            np.save(out / "scan_hits.npy", np.concatenate(preview))
        if not updates:
            raise RuntimeError("no scan-matched poses")
        csv_save(out / "updates.csv", [[t, *v] for t, v in sorted(updates.items())])
        csv_save(out / "odometry.csv", odom_rows)
        dataset.stats.update(forwarded=len(odom_rows), pose_updates=len(updates))
    node.destroy_node()
    return modes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["map", "localize"])
    parser.add_argument("bag", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--lidar", help="override lidar topic from profile")
    parser.add_argument("--map", type=Path)
    parser.add_argument("--initial", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--refine", action="store_true", help="offline bidirectional scan matching and SE(2) smoothing")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()
    if args.refine and args.mode != "localize":
        parser.error("--refine requires localize")
    if args.rate <= 0 or not math.isfinite(args.rate):
        parser.error("--rate must be positive")
    if args.mode == "localize":
        if not (args.map and args.map.is_file()):
            parser.error("localize requires --map with an existing map YAML")
        load_amcl_map(args.map)
    profile = yaml.safe_load((args.profile or Path("/mgts/configs/go2.yaml")).read_text())
    manifest = args.bag / "manifest.json"
    if manifest.is_file() and (args.bag / "sensors").is_dir():
        capture = json.loads(manifest.read_text())
        if not capture["complete"]:
            parser.error(f"incomplete capture: {args.bag}")
        if args.profile is None:
            profile["lidar"] = capture["lidar"]
        args.bag = args.bag / "sensors"
    if args.lidar:
        profile["lidar"] = args.lidar
    rclpy.init()
    try:
        dataset = Dataset(args.bag, profile)
        scans = dataset.scans()
        first = next(scans, None)
        if first is None:
            raise ValueError(f"no usable scans: {dataset.stats}")
        args.out.mkdir(parents=True, exist_ok=False)
        modes = run(args, args.out, dataset, first, scans)
        config_name = "mapping.yaml" if args.mode == "map" else "amcl.yaml"
        copyfile(Path("/mgts/configs", config_name), args.out / config_name)
        report = dict(
            mode=args.mode,
            refine=args.refine,
            profile=profile,
            bag=str(args.bag),
            initial=args.initial,
            map=str(args.map.resolve()) if args.map else None,
            scan_window=dataset.scan_window,
            counts=dict(dataset.stats),
        )
        if args.refine:
            report["refinement_segments"] = modes
        if dataset.imu_offsets:
            report["scan_to_nearest_imu_p95_s"] = float(np.percentile(dataset.imu_offsets, 95))
        (args.out / "run.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(dataset.stats))
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
