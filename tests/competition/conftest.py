from pathlib import Path
import copy
import numpy as np
import pytest
from competition.config import load
from competition import bagio,cloud

@pytest.fixture
def config():
 c=load(Path(__file__).resolve().parents[2]/'configs/competition/dataset.yaml')
 c['topics']={'lidar':'/points','imu':'/imu','odometry':None}
 c['extrinsics']={'T_base_lidar':[.2,0,0,0,0,0,1.], 'T_base_imu':[0.,0.,0.,0.,0.,0.,1.]}
 c['units']={'xyz':'m','acceleration':'m/s2','angular_velocity':'rad/s'}
 c['point_time']={'mode':'relative','unit':'s','field':'time','offset_s':0.}
 c['localization']['initial_pose']=[0.,0.,0.,0.,0.,0.,1.]
 return c

def imu(n,gyro=1.,acc=9.80665):
 T=bagio.STORE.types
 return T['sensor_msgs/msg/Imu'](bagio.header(n,'imu'), T['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.),
   np.array([-1.]+[0.]*8),T['geometry_msgs/msg/Vector3'](0.,0.,float(gyro)),np.ones(9),
   T['geometry_msgs/msg/Vector3'](0.,0.,float(acc)),np.ones(9))

def make_bag(path,*,count=10,point_times=True,reverse=False,mcap=False):
 from rosbags.rosbag2 import Writer,StoragePlugin
 start=1700000000000000000
 xyz=np.array([[2.,j*.1,.8] for j in range(20)])
 records=[]
 for j in range(count*10):
  n=start+j*10000000;records.append(('/imu',n,imu(n)))
  if j%10==0:
   stamp=n if not reverse or j!=10 else start
   msg=cloud.pack(xyz,stamp,'lidar',times=np.linspace(0,.09,len(xyz)) if point_times else None)
   records.append(('/points',n,msg))
 records.sort(key=lambda a:a[1])
 with Writer(path,version=9,storage_plugin=StoragePlugin.MCAP if mcap else StoragePlugin.SQLITE3) as w:
  cs={}
  for topic,n,msg in records:
   if topic not in cs:cs[topic]=w.add_connection(topic,msg.__msgtype__,typestore=bagio.STORE)
   w.write(cs[topic],n,bagio.STORE.serialize_cdr(msg,msg.__msgtype__))
 return path
