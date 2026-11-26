#!/usr/bin/env python3
"""Publish saved output to RViz. Run inside the ROS image; no SLAM is performed."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition import pcd, bagio, cloud
from competition.trajectory import read_tum
from competition.config import load

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as PathMsg, OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    parser = argparse.ArgumentParser(); parser.add_argument('results'); parser.add_argument('--max-points', type=int, default=500000)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:]); root = Path(args.results)
    loc_config = root/'localization/config_resolved.yaml'
    c = load(loc_config if loc_config.is_file() else root/'mapping/config_resolved.yaml')
    frame = c['frames']['map']
    prior_map = c['localization'].get('prior_map')
    prior_trajectory = c['localization'].get('prior_trajectory')
    rclpy.init(); node = Node('competition_result_viewer')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    outputs = []
    def add(typ, topic, msg):
        outputs.append((node.create_publisher(typ, topic, qos), msg))
    points = pcd.xyz(prior_map or root/'mapping/map.pcd')
    if args.max_points <= 0:
        raise ValueError('--max-points must be positive')
    points = points[::max(1, (len(points)+args.max_points-1)//args.max_points)]
    msg = cloud.pack(points, 0, frame)
    add(PointCloud2, '/competition/result/map', deserialize_message(bytes(bagio.STORE.serialize_cdr(msg, msg.__msgtype__)), PointCloud2))
    mapping_trajectory = Path(prior_trajectory) if prior_trajectory else root/'mapping/trajectory_optimized.tum'
    for topic, file in [('mapping',mapping_trajectory), ('localization',root/'localization/trajectory.tum')]:
        if not file.exists():
            continue
        path = PathMsg(); path.header.frame_id = frame
        for row in read_tum(file):
            pose = PoseStamped(); pose.header.frame_id = frame
            pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(round(row[0]*1e9), 1000000000)
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = map(float,row[1:4])
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = map(float,row[4:])
            path.poses.append(pose)
        add(PathMsg,'/competition/result/'+topic,path)
    if (root/'occupancy/occupancy_bag').exists():
        with bagio.reader(root/'occupancy/occupancy_bag') as r:
            for conn, _, raw in r.messages():
                if conn.msgtype == 'nav_msgs/msg/OccupancyGrid':
                    add(OccupancyGrid,'/competition/result/occupancy',deserialize_message(bytes(raw),OccupancyGrid)); break
    def publish():
        for pub, msg in outputs:
            pub.publish(msg)
    node.create_timer(1.,publish); publish()
    print('Publishing results. RViz fixed frame: '+frame, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
