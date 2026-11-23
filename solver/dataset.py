"""Read only the sensor topics and local TF edges selected by the input profile."""

import math
from collections import Counter

import numpy as np
import rosbag2_py
from geometry import compose, inverse
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import Buffer, TransformException


def messages(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = set(topics) - types.keys()
    if missing:
        raise ValueError(f"bag is missing {missing}; contains {sorted(types)}")
    classes = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, classes[topic])


def xyz(v):
    return [v.x, v.y, v.z]


def rotation(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


def planar(t):
    forward = rotation(t.rotation).as_matrix()[:, 0]
    return [t.translation.x, t.translation.y, math.atan2(forward[1], forward[0])]


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Dataset:
    def __init__(self, path, profile):
        self.path, self.profile = path, profile
        self.stats = Counter()
        self.buffer = Buffer(cache_time=Duration(seconds=86400))
        self.edges = {tuple(pair) for pair in profile["tf_edges"]}
        odom_edge = (profile["odom_frame"], profile["base_frame"])
        odometry = profile.get("odometry")
        if odometry and odom_edge in self.edges:
            raise ValueError("choose one source for the odometry transform")
        if not odometry and odom_edge not in self.edges:
            raise ValueError("select odometry topic or odometry TF edge")
        topics = [profile.get("tf_topic", "/tf"), profile.get("static_topic", "/tf_static")]
        topics = [t for t in topics if t]
        if odometry:
            topics.append(odometry)
        imu_topic = profile.get("imu")
        imu_times = []
        if imu_topic:
            topics.append(imu_topic)
        for edge in profile.get("calibrations", []):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = edge["frames"]
            if tuple(edge["frames"]) in self.edges or tuple(edge["frames"]) == odom_edge:
                raise ValueError(f"duplicate TF source for {edge['frames']}")
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(
                float, edge["xyz"]
            )
            q = tf.transform.rotation
            q.x, q.y, q.z, q.w = Rotation.from_euler("xyz", edge["rpy"]).as_quat().tolist()
            self.buffer.set_transform_static(tf, "profile")
        for topic, msg in messages(path, topics):
            if topic == imu_topic:
                if not isinstance(msg, Imu):
                    raise ValueError(f"{topic}: expected sensor_msgs/msg/Imu")
                imu_times.append(stamp(msg.header) + profile.get("imu_time_offset_s", 0.0))
                continue
            if topic == odometry:
                if (msg.header.frame_id, msg.child_frame_id) != odom_edge:
                    raise ValueError(
                        f"unexpected odometry frames {msg.header.frame_id}, {msg.child_frame_id}"
                    )
                tf = TransformStamped(header=msg.header, child_frame_id=msg.child_frame_id)
                p = msg.pose.pose.position
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz(p)
                tf.transform.rotation = msg.pose.pose.orientation
                self.buffer.set_transform(tf, "kinematics")
            else:
                for tf in msg.transforms:
                    edge = (tf.header.frame_id, tf.child_frame_id)
                    if edge in self.edges:
                        if topic == profile.get("static_topic", "/tf_static"):
                            self.buffer.set_transform_static(tf, "calibration")
                        else:
                            self.buffer.set_transform(tf, "local")
        self.imu_times = np.unique(imu_times)
        self.imu_offsets = []
        self.stats["imu_samples"] = len(imu_times)
        self.stats["duplicate_imu_stamps"] = len(imu_times) - len(self.imu_times)
        self.offset = None
        self.scan_window = None

    def scans(self):
        p = self.profile
        lidar = p["lidar"]
        previous = -math.inf
        last_pose = None
        last_t = None
        keep = int(p.get("keep_every_scan", 1))
        if (
            keep < 1
            or p.get("keep_every_point", 1) < 1
            or p["bins"] < 3
            or not 0 <= p["range"][0] < p["range"][1]
        ):
            raise ValueError("invalid lidar sampling or range")
        for _, msg in messages(self.path, [lidar]):
            self.stats["input_scans"] += 1
            t = stamp(msg.header)
            if t <= previous:
                self.stats["nonmonotonic"] += 1
                continue
            previous = t
            if self.scan_window is None:
                self.scan_window = [t, t]
            self.scan_window[1] = t
            if (self.stats["input_scans"] - 1) % keep:
                self.stats["dropped"] += 1
                continue
            if self.imu_times.size:
                idx = np.searchsorted(self.imu_times, t)
                near = self.imu_times[max(0, idx - 1) : idx + 1]
                self.imu_offsets.append(float(np.min(abs(near - t))))
            when = Time.from_msg(msg.header.stamp)
            try:
                odom = self.buffer.lookup_transform(p["odom_frame"], p["base_frame"], when)
                extrinsic = self.buffer.lookup_transform(p["base_frame"], msg.header.frame_id, when)
            except TransformException as err:
                if not self.stats["missing_tf"]:
                    print("First missing TF:", err, flush=True)
                self.stats["missing_tf"] += 1
                continue
            pose = planar(odom.transform)
            tilt = Rotation.from_euler("z", -pose[2]) * rotation(odom.transform.rotation)
            scan = self.project(msg, extrinsic.transform, tilt)
            if np.count_nonzero(np.isfinite(scan.ranges)) < 3:
                self.stats["empty"] += 1
                continue
            if last_pose is None:
                stressed = pose
            else:
                delta = compose(inverse(last_pose), pose)
                delta[:2] *= p.get("odom_scale", 1.0)
                delta[2] += p.get("yaw_drift_rad_s", 0.0) * (t - last_t)
                stressed = compose(stressed, delta)
            last_pose, last_t = pose, t
            yield scan, stressed

    def project(self, msg, extrinsic, tilt):
        p = self.profile
        if isinstance(msg, LaserScan):
            ranges = np.asarray(msg.ranges)
            angle = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            r, a = ranges[valid], angle[valid]
            points = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r)))
        elif isinstance(msg, PointCloud2):
            cloud = read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
            points = np.column_stack([cloud[n].ravel() for n in ("x", "y", "z")])
            points = points[np.isfinite(points).all(axis=1)]
        else:
            raise TypeError(f"expected LaserScan or PointCloud2, got {type(msg).__name__}")
        mount = tilt.apply(xyz(extrinsic.translation))
        points = points[:: p.get("keep_every_point", 1)]
        points = points @ (tilt * rotation(extrinsic.rotation)).as_matrix().T + mount
        if not isinstance(msg, LaserScan):
            lo, hi = p["cloud_height"]
            points = points[(points[:, 2] >= lo) & (points[:, 2] <= hi)]
        if self.offset is None:
            self.offset = mount[:2].copy()
        xy = points[:, :2] - self.offset
        r = np.linalg.norm(xy, axis=1)
        lo, hi = p["range"]
        valid = (r >= lo) & (r <= hi)
        n = p["bins"]
        step = 2 * math.pi / n
        bins = np.rint((np.arctan2(xy[valid, 1], xy[valid, 0]) + math.pi) / step).astype(int) % n
        ranges = np.full(n, np.inf, dtype=np.float32)
        np.minimum.at(ranges, bins, r[valid])
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = "laser"
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi - step, step
        scan.range_min, scan.range_max = float(lo), float(hi)
        scan.ranges = ranges.tolist()
        return scan
