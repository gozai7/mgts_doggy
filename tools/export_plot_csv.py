#!/usr/bin/env python3
"""Stable scalar column names for PlotJuggler; blank means unavailable, not zero."""
import argparse
import csv
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition import bagio

def export(bag, output, diagnostics=None):
    columns=['t','gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z','imu_dt_s','lidar_dt_s',
             'odom_x','odom_y','odom_z','fitness_score','alignment_time_s','accepted']
    rows=[]; previous={}
    with bagio.reader(bag) as r:
        for conn, _, raw in r.messages():
            if conn.topic not in ('/competition/lidar','/competition/imu','/competition/odometry'):
                continue
            msg=r.deserialize(raw,conn.msgtype);n=bagio.ns(msg);row={'t':n*1e-9}
            dt=(n-previous[conn.topic])*1e-9 if conn.topic in previous else None
            previous[conn.topic]=n
            if conn.topic.endswith('imu'):
                row.update(zip(['gyro_x','gyro_y','gyro_z'],bagio.vec(msg.angular_velocity)))
                row.update(zip(['accel_x','accel_y','accel_z'],bagio.vec(msg.linear_acceleration)))
                row['imu_dt_s']=dt
            elif conn.topic.endswith('lidar'):
                row['lidar_dt_s']=dt
            else:
                row.update(zip(['odom_x','odom_y','odom_z'],bagio.vec(msg.pose.pose.position)))
            rows.append(row)
    if diagnostics:
        for line in Path(diagnostics).read_text().splitlines():
            d=json.loads(line);v=d['values']
            rows.append({'t':d['stamp_ns']*1e-9,'fitness_score':v.get('fitness_score',''),
                         'alignment_time_s':v.get('alignment_time_sec',''),
                         'accepted':int(d['message'] in ('ok','alternate_seed_recovered','recovery_retry_from_last_pose_recovered'))})
    rows.sort(key=lambda r:r['t'])
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(rows)
    return len(rows)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('normalized_bag');p.add_argument('output');p.add_argument('--diagnostics')
    a=p.parse_args();print(export(a.normalized_bag,a.output,a.diagnostics))
