"""Create a deterministic ROS sensor fixture, or score a completed offline replay."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

DT, EVENT, END = 0.1, 25., 115.
RES, WIDTH, HEIGHT = 0.05, 12., 9.
RADIUS, OMEGA = 1.2, 0.35
ROOM = (0.1, 0.1, 11.9, 8.9)
BLOCKS = [(4.8, 0.1, 5.0, 4.5), (1.0, 5.4, 3.2, 5.8),
          (5.9, 6.7, 6.6, 7.8), (9.85, 4.1, 10.4, 6.1)]


def write_map(path):
    rows, cols = np.indices((round(HEIGHT / RES), round(WIDTH / RES)))
    x, y = (cols + .5) * RES, (rows + .5) * RES

    def inside(box):
        x0, y0, x1, y1 = box
        return (x > x0) & (x < x1) & (y > y0) & (y < y1)

    free = inside(ROOM)
    for box in BLOCKS:
        free &= ~inside(box)
    Image.fromarray(np.where(free[::-1], 254, 0).astype('uint8')).save(path / 'map.pgm')
    (path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', mode='trinary',
        resolution=RES, origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))


def cast(x, y, directions):
    dx, dy = np.cos(directions), np.sin(directions)
    best = np.full(len(dx), np.inf)
    for x0, y0, x1, y1 in [ROOM, *BLOCKS]:
        for wall, origin, along, cross, other, lo, hi in [
            (x0, x, dx, dy, y, y0, y1), (x1, x, dx, dy, y, y0, y1),
            (y0, y, dy, dx, x, x0, x1), (y1, y, dy, dx, x, x0, x1),
        ]:
            t = np.divide(wall - origin, along, out=np.zeros_like(dx), where=abs(along) > 1e-12)
            hit = other + t * cross
            best = np.minimum(best, np.where((t > 0) & (hit >= lo) & (hit <= hi), t, np.inf))
    assert np.all((best >= .12) & (best <= 16.))
    return best.tolist()


def generate(directory):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan

    sensor_dir = directory / 'input'
    sensor_dir.mkdir()
    truth_dir = directory / 'truth'
    truth_dir.mkdir()
    write_map(sensor_dir)
    profile = yaml.safe_load(Path('/mgts/configs/go2.yaml').read_text())
    profile.update(tf_topic=None, static_topic=None, imu=None)
    (sensor_dir / 'profile.yaml').write_text(yaml.safe_dump(profile))
    cfg = yaml.safe_load(Path('/mgts/configs/amcl.yaml').read_text())
    cfg['amcl']['ros__parameters'].update(recovery_alpha_fast=.1, recovery_alpha_slow=.001)
    (sensor_dir / 'recovery.yaml').write_text(yaml.safe_dump(cfg))
    angles = np.deg2rad(np.arange(-180, 180))
    for scenario in ['clean', 'jump']:
        bag = rosbag2_py.SequentialWriter()
        bag.open(rosbag2_py.StorageOptions(uri=str(sensor_dir / scenario), storage_id='mcap'),
                 rosbag2_py.ConverterOptions('', ''))
        for topic, kind in [('/scan', 'sensor_msgs/msg/LaserScan'), ('/odom', 'nav_msgs/msg/Odometry')]:
            bag.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
        truth = []
        for i in range(round(END / DT) + 1):
            elapsed = i * DT
            ns = 1_000_000_000 + i * 100_000_000
            stamp = Time(nanoseconds=ns).to_msg()
            phase = OMEGA * elapsed
            x = 2.5 + RADIUS * np.cos(phase)
            y = 2.5 + RADIUS * np.sin(phase)
            if scenario == 'jump' and elapsed >= EVENT:
                x += 5.4
                y += 3.5
            yaw = np.pi / 2 + phase
            truth.append([ns * 1e-9, x, y, yaw])
            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_footprint'
            od.pose.pose.position.x = RADIUS * np.sin(phase)
            od.pose.pose.position.y = RADIUS * (1 - np.cos(phase))
            od.pose.pose.orientation.z = np.sin(phase / 2)
            od.pose.pose.orientation.w = np.cos(phase / 2)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
            scan.angle_min, scan.angle_increment, scan.angle_max = angles[0], np.pi / 180, angles[-1]
            scan.range_min, scan.range_max = .12, 16.
            scan.ranges = cast(x, y, angles + yaw)
            for topic, message in [('/odom', od), ('/scan', scan)]:
                bag.write(topic, serialize_message(message), ns)
        bag.close()
        np.savetxt(truth_dir / f'{scenario}.csv', truth, delimiter=',', header='t,x,y,yaw', comments='')


def score(truth_path, run):
    truth = np.loadtxt(truth_path, delimiter=',', skiprows=1)
    estimate = np.loadtxt(run / 'trajectory.csv', delimiter=',', skiprows=1)
    updates = np.loadtxt(run / 'updates.csv', delimiter=',', skiprows=1, ndmin=2)
    assert truth.shape == estimate.shape and np.allclose(truth[:, 0], estimate[:, 0], atol=1e-7, rtol=0)
    t = np.round(truth[:, 0] - truth[0, 0], 6)
    xy = np.linalg.norm(estimate[:, 1:3] - truth[:, 1:3], axis=1)
    yaw = abs(np.rad2deg(np.angle(np.exp(1j * (estimate[:, 3] - truth[:, 3])))))
    pre = (t >= EVENT - 5) & (t < EVENT)
    good = (xy <= .5) & (yaw <= 15)
    hold = round(5. / DT) + 1
    first = next((i for i in np.flatnonzero(t >= EVENT) if i + hold <= len(t) and good[i:i + hold].all()), None)
    result = dict(updates=len(updates), poses=len(estimate), pre_rmse_m=float(np.sqrt(np.mean(xy[pre]**2))),
                  pre_yaw_max_deg=float(yaw[pre].max()), pre_xy_max_m=float(xy[pre].max()),
                  first_post_error_m=float(xy[t == EVENT][0]), post_min_m=float(xy[t >= EVENT].min()),
                  final_m=float(xy[-1]),
                  final_yaw_deg=float(yaw[-1]), post_observation_s=END-EVENT,
                  first_stable_start_s=None if first is None else float(t[first]-EVENT),
                  held_to_end=None if first is None else bool(good[first:].all()))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    cmds = p.add_subparsers(dest='command', required=True)
    cmds.add_parser('generate').add_argument('directory', type=Path)
    scorer = cmds.add_parser('score')
    scorer.add_argument('truth', type=Path)
    scorer.add_argument('run', type=Path)
    args = p.parse_args()
    if args.command == 'generate':
        generate(args.directory)
    else:
        score(args.truth, args.run)
