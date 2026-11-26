import copy,json,hashlib
from pathlib import Path
import numpy as np
import pytest
from competition import bagio,cloud
from competition.normalize import normalize_bag
from competition.doctor import audit
from competition.perturb import perturb
from competition.config import validate
from conftest import make_bag

@pytest.mark.parametrize('mcap',[False,True])
def test_normalize_real_serialized_bag(tmp_path,config,mcap):
 src=make_bag(tmp_path/'input',mcap=mcap)
 before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in src.iterdir() if p.is_file()}
 config['time_offsets_s']['imu']=.02
 config['units']['acceleration']='g';config['units']['angular_velocity']='deg/s'
 r=normalize_bag(src,tmp_path/'out',config)
 assert r['counts']=={'imu':100,'lidar':10};assert r['complete'];assert r['original_modified'] is False
 with bagio.reader(tmp_path/'out/bag') as reader:
  names={c.topic for c in reader.connections};assert names=={'/competition/imu','/competition/lidar','/tf_static'}
  previous=0
  for conn,t,raw in reader.messages():
   assert t>=previous;previous=t
   m=reader.deserialize(raw,conn.msgtype)
   if conn.topic.endswith('imu'):
    assert np.isclose(m.angular_velocity.z,np.pi/180)
    assert np.isclose(m.linear_acceleration.z,9.80665**2)
 after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in src.iterdir() if p.is_file()}
 assert before==after
 with pytest.raises(FileExistsError):normalize_bag(src,tmp_path/'out',config)

def test_nonmonotonic_bag_fails(tmp_path,config):
 src=make_bag(tmp_path/'bad',reverse=True)
 with pytest.raises(ValueError,match='decreasing'):normalize_bag(src,tmp_path/'out',config)
 assert not (tmp_path/'out').exists();assert (tmp_path/'out.partial').is_dir()
 r=audit(src,tmp_path/'doctor');assert any('decreasing' in e for e in r['errors'])

def test_unknown_calibration_fails(config):
 config['extrinsics']['T_base_lidar']=None
 with pytest.raises(ValueError,match='Missing measured'):validate(config)

def test_diagnostic_no_point_time(tmp_path):
 r=audit(make_bag(tmp_path/'bag',point_times=False),tmp_path/'doctor')
 assert r['complete_header_audit'];assert r['topics']['/points']['rate_hz']==pytest.approx(10)
 assert r['topics']['/points']['sampled_clouds'][0]['point_time_candidates']=={}
 assert r['tf_edges']==[];assert r['assumptions'];assert r['sensor_overlap_s']>0

def test_sample_bound_is_reported(tmp_path):
 r=audit(make_bag(tmp_path/'bag'),tmp_path/'doctor',max_messages=5)
 assert not r['complete_header_audit'];assert r['read_messages']==5

def read_selected(path,topic):
 with bagio.reader(path) as r:
  return [(n,r.deserialize(raw,c.msgtype)) for c,n,raw in r.messages(connections=[c for c in r.connections if c.topic==topic])]

def test_fault_clocks_separate_and_seed_reproducible(tmp_path):
 src=make_bag(tmp_path/'bag')
 kw=dict(topic='/points',seed=10,header_shift_s=.02,record_shift_s=.04,point_shift=.01,keep_points=.5)
 perturb(src,tmp_path/'a',**kw);perturb(src,tmp_path/'b',**kw)
 original=read_selected(src,'/points');a=read_selected(tmp_path/'a/bag','/points');b=read_selected(tmp_path/'b/bag','/points')
 assert len(a)==len(original)
 for (n,m),(n0,m0),(n2,m2) in zip(a,original,b):
  assert n-n0==40000000;assert bagio.ns(m)-bagio.ns(m0)==20000000
  np.testing.assert_array_equal(m.data,m2.data);assert m.width<=m0.width
  assert np.all(cloud.fields(m)['time']>=.01)
 assert json.loads((tmp_path/'a/manifest.json').read_text())['original_modified'] is False

def test_fault_removes_timestamp_field(tmp_path):
 src=make_bag(tmp_path/'bag');perturb(src,tmp_path/'a',topic='/points',remove_time=True)
 _,m=read_selected(tmp_path/'a/bag','/points')[0];assert 'time' not in cloud.fields(m).dtype.names

def test_fault_never_overwrites_source(tmp_path):
 src=make_bag(tmp_path/'bag')
 with pytest.raises(ValueError):perturb(src,src,topic='/points')
 with pytest.raises(ValueError):perturb(src,src/'inside',topic='/points')
