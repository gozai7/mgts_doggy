#!/usr/bin/env python3
"""Small asymmetric room, synthetic sensor messages. NOT a SLAM accuracy benchmark."""
import argparse
import copy
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from competition import bagio, cloud
from competition.config import load, transform, pose7
from competition.trajectory import save

def motion(t, duration):
 u=np.clip((t-3)/(duration-3),0,1)
 phase=2*np.pi*(3*u*u-2*u*u*u)
 return np.array([1.5*np.sin(phase),1-np.cos(phase),0.]), .3*np.sin(phase)

def generate(output, duration=20., seed=8, mapping_mode="cpu", with_odometry=False):
 output=Path(output);output.mkdir(parents=True,exist_ok=False)
 rng=np.random.default_rng(seed); T=bagio.STORE.types
 # Walls plus an asymmetric cabinet; angular ray intersections in XY.
 segments=[((-4,-3),(5,-3)),((5,-3),(5,4)),((5,4),(-4,4)),((-4,4),(-4,-3)),
           ((2.8,.8),(3.5,.8)),((3.5,.8),(3.5,1.5)),((3.5,1.5),(2.8,1.5)),((2.8,1.5),(2.8,.8))]
 base_ns=1700000000000000000
 for phase in ('mapping','localization'):
  records=[]; truth=[]; first_pose=None
  for i in range(round(duration*100)+1):
   if phase=='localization' and i < round(duration*45):continue
   t=i/100; n=base_ns+round(t*1e9);pos,yaw=motion(t,duration)
   h=.001;p0,y0=motion(t-h,duration);p1,y1=motion(t+h,duration)
   acc=(p1-2*pos+p0)/(h*h)+np.array([0,0,9.80665])
   omega=(y1-y0)/(2*h)
   R=Rotation.from_euler('z',yaw)
   imu=T['sensor_msgs/msg/Imu'](bagio.header(n,'imu'),T['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.),
       np.array([-1.]+[0.]*8), T['geometry_msgs/msg/Vector3'](0.,0.,float(omega)),np.zeros(9),
       T['geometry_msgs/msg/Vector3'](*map(float,R.inv().apply(acc))),np.zeros(9))
   records.append(('/imu/data',n,imu))
   if with_odometry:
    physical=transform([*pos,*R.as_quat()])
    if first_pose is None:first_pose=physical.copy(); first_t=t
    relative=np.linalg.inv(first_pose)@physical
    relative[:3,3]+=np.array([.004,-.001,0.])*(t-first_t)
    relative[:3,:3]=relative[:3,:3]@Rotation.from_euler('z',.0005*(t-first_t)).as_matrix()
    values=pose7(relative)
    p=T['geometry_msgs/msg/Pose'](T['geometry_msgs/msg/Point'](*map(float,values[:3])),T['geometry_msgs/msg/Quaternion'](*map(float,values[3:])))
    # Simulated noisy relative odometry, never the global GT pose.
    velocity=relative[:3,:3].T@(first_pose[:3,:3].T@((p1-p0)/(2*h))+np.array([.004,-.001,0.]))
    tw=T['geometry_msgs/msg/Twist'](T['geometry_msgs/msg/Vector3'](*map(float,velocity)),T['geometry_msgs/msg/Vector3'](0.,0.,float(omega+.0005)))
    odom=T['nav_msgs/msg/Odometry'](bagio.header(n,'odom'),'base_link',T['geometry_msgs/msg/PoseWithCovariance'](p,np.eye(6).ravel()*.01),T['geometry_msgs/msg/TwistWithCovariance'](tw,np.eye(6).ravel()*.01))
    records.append(('/robot/odometry',n,odom))
   if i%10:continue
   angles=np.linspace(-np.pi,np.pi,360,endpoint=False)+yaw
   dirs=np.column_stack((np.cos(angles),np.sin(angles)))
   dist=np.full(len(dirs),np.inf)
   for aa,bb in segments:
    aa,bb=np.array(aa,dtype=float),np.array(bb,dtype=float);v=bb-aa;w=aa-pos[:2]
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    den=cross(dirs,v)
    with np.errstate(divide='ignore',invalid='ignore'):
     s=cross(w,v)/den;u=cross(w,dirs)/den
    valid=(s>0)&(u>=0)&(u<=1);dist=np.minimum(dist,np.where(valid,s,np.inf))
   xy=dirs*dist[:,None];world=[]
   for z in [.2,.6,1.,1.4]:world.append(np.column_stack((xy,np.full(len(xy),z))))
   pts=R.inv().apply(np.vstack(world))+rng.normal(0,.003,(1440,3))
   m=cloud.pack(pts,n,'lidar')
   records.append(('/lidar/points',n,m))
   truth.append([n*1e-9,*pos,*R.as_quat()])
  records.sort(key=lambda x:x[1]);bagio.write_messages(output/phase,records)
  save(output/(phase+'_truth.tum'),np.array(truth))
 c=load(Path(__file__).resolve().parents[1]/'configs/competition/dataset.yaml')
 c['bags']={k:str((output/k).resolve()) for k in ('mapping','localization')}
 c['output']=str((output/'results').resolve());c['topics']={'lidar':'/lidar/points','imu':'/imu/data','odometry':None}
 c['extrinsics']={'T_base_lidar':[0.,0.,0.,0.,0.,0.,1.],'T_base_imu':[0.,0.,0.,0.,0.,0.,1.]}
 c['units']={'xyz':'m','acceleration':'m/s2','angular_velocity':'rad/s'}
 c['point_time']={'field':None,'unit':None,'mode':'none','offset_s':0.}
 c['localization']['initial_pose']=None
 if with_odometry:
  c['topics']['odometry']='/robot/odometry'
  c['source_frames']['odometry']='odom';c['source_frames']['odometry_child']='base_link'
  c['localization']['prediction']='kinematic'
  c['odometry']={'gravity_aligned':True,'initial_roll_pitch_rad':[0.,0.]}
  c['beluga']={'initial_base_height_m':0.,'seed':42}
 c['occupancy']['floor_z_m']=-.4
 c['mapping']['mode']=mapping_mode
 c['mapping']['min_trajectory_coverage']=.6
 (output/'dataset.yaml').write_text(yaml.safe_dump(c))
 return output/'dataset.yaml'

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--duration',type=float,default=20.);p.add_argument('--mapping-mode',choices=('cpu','gpu'),default='cpu');p.add_argument('--with-odometry',action='store_true');a=p.parse_args()
 if a.duration<=5:raise ValueError('duration must exceed 5 seconds')
 print(generate(a.output,a.duration,mapping_mode=a.mapping_mode,with_odometry=a.with_odometry))
