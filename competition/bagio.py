"""ROS-free rosbag2 (SQLite3/MCAP) I/O via rosbags 0.11."""
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

STORE = get_typestore(Stores.ROS2_JAZZY)


def reader(path):
    return AnyReader([Path(path)], default_typestore=STORE)


def ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def time_msg(n):
    sec, nanosec = divmod(int(n), 1_000_000_000)
    return STORE.types["builtin_interfaces/msg/Time"](sec=sec, nanosec=nanosec)


def header(n, frame):
    return STORE.types["std_msgs/msg/Header"](stamp=time_msg(n), frame_id=frame)


def set_stamp(msg, n):
    msg.header.stamp = time_msg(n)


def vec(msg):
    return np.array([msg.x, msg.y, msg.z], dtype=float)


def assign_vec(msg, values):
    msg.x, msg.y, msg.z = map(float, values)


def pose_values(p):
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def tf_message(n, parent, child, values):
    xyz, q = values[:3], values[3:]
    t = STORE.types["geometry_msgs/msg/Transform"](
        translation=STORE.types["geometry_msgs/msg/Vector3"](*map(float, xyz)),
        rotation=STORE.types["geometry_msgs/msg/Quaternion"](*map(float, q)))
    return STORE.types["geometry_msgs/msg/TransformStamped"](
        header=header(n, parent), child_frame_id=child, transform=t)


def write_messages(path, records):
    """records: (topic, ns, message), already ordered by ns."""
    with Writer(path, version=9) as w:
        connections = {}
        for topic, n, msg in records:
            typ = msg.__msgtype__
            key = (topic, typ)
            if key not in connections:
                connections[key] = w.add_connection(topic, typ, typestore=STORE)
            w.write(connections[key], int(n), STORE.serialize_cdr(msg, typ))
