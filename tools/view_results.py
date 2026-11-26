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
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as PathMsg, OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    parser = argparse.ArgumentParser(); parser.add_argument('results'); parser.add_argument('--max-points', type=int, default=500000)
    parser.add_argument('--flat', action='store_true', help='Read map.pcd, mapping.tum and trajectory.tum in the map frame')
    args = parser.parse_args(); root = Path(args.results)
    if args.flat:
        frame = 'map'
        map_file = root/'map.pcd'
        paths = [('mapping', root/'mapping.tum')]
        if (root/'full_route.tum').is_file():
            paths.append(('full_route', root/'full_route.tum'))
        if (root/'trajectory.tum').is_file():
            paths.append(('localization', root/'trajectory.tum'))
        if (root/'glim_b_in_map.tum').is_file():
            paths.append(('glim_b', root/'glim_b_in_map.tum'))
    else:
        frame = load(root/'mapping/config_resolved.yaml')['frames']['map']
        map_file = root/'mapping/map.pcd'
        paths = [('mapping', root/'mapping/trajectory_optimized.tum'), ('localization', root/'localization/trajectory.tum')]
    rclpy.init(); node = Node('competition_result_viewer')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    outputs = []
    def add(typ, topic, msg):
        outputs.append((node.create_publisher(typ, topic, qos), msg))
    points = pcd.xyz(map_file)
    if args.max_points <= 0:
        raise ValueError('--max-points must be positive')
    points = points[::max(1, (len(points)+args.max_points-1)//args.max_points)]
    msg = cloud.pack(points, 0, frame)
    add(PointCloud2, '/competition/result/map', deserialize_message(bytes(bagio.STORE.serialize_cdr(msg, msg.__msgtype__)), PointCloud2))
    for topic, file in paths:
        if not args.flat and not file.exists():
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
    if not args.flat:
        node.create_timer(1.,publish)
    publish()
    print('Publishing results. RViz fixed frame: '+frame, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
