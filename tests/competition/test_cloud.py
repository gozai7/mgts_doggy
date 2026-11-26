import copy
import numpy as np
import pytest
from competition import cloud,bagio
from competition.config import transform,phase_config

@pytest.mark.parametrize('endian',['<','>'])
def test_padding_endianness_and_multicount(endian):
 names=['x','y','z','normal','t'];dtype=np.dtype({'names':names,'formats':[endian+'f4']*3+[(endian+'f4',(3,)),endian+'u4'],
   'offsets':[0,4,8,12,24],'itemsize':32})
 raw=bytearray(2*80);arr=np.ndarray((2,2),dtype=dtype,buffer=raw,strides=(80,32))
 arr['x']=[[1,2],[3,4]];arr['y']=5;arr['z']=6;arr['t']=1000000
 m=cloud.pack(np.zeros((4,3)),100,'sensor')
 m.height=2;m.width=2;m.row_step=80;m.point_step=32;m.is_bigendian=endian=='>'
 F=bagio.STORE.types['sensor_msgs/msg/PointField']
 m.fields=[F('x',0,7,1),F('y',4,7,1),F('z',8,7,1),F('normal',12,7,3),F('t',24,6,1)]
 m.data=np.frombuffer(raw,dtype=np.uint8)
 xyz,fields=cloud.unpack(m)
 np.testing.assert_array_equal(xyz[:,0],[1,2,3,4]);assert fields['normal'].shape==(4,3)
 np.testing.assert_allclose(cloud.point_seconds(fields,100,{'mode':'relative','unit':'ns','field':'t'},.1),.001)

@pytest.mark.parametrize('unit,scale',[('s',1),('ms',1000),('us',1e6),('ns',1e9)])
def test_point_time_units(unit,scale):
 arr=np.zeros(3,dtype=[('offset_time','f8')]);arr['offset_time']=np.array([0,.05,.1])*scale
 np.testing.assert_allclose(cloud.point_seconds(arr,100,{'mode':'relative','unit':unit,'field':'offset_time'},.1),[0,.05,.1])

def test_absolute_time_sign():
 arr=np.array([(1000.,),(1000.05,)],dtype=[('timestamp','f8')])
 np.testing.assert_allclose(cloud.point_seconds(arr,1000000000000,{'mode':'absolute','unit':'s','field':'timestamp'},.1),[0,.05])

def test_missing_time_requires_explicit_none():
 arr=np.zeros(3,dtype=[('x','f4')])
 with pytest.raises(ValueError,match='missing'):cloud.point_seconds(arr,10,{'mode':'relative','unit':'s','field':'time'},.1)
 assert cloud.point_seconds(arr,10,{'mode':'none'},.1) is None

def test_wrong_units_fail():
 arr=np.array([(100000.,)],dtype=[('time','f8')])
 with pytest.raises(ValueError,match='Invalid point times'):cloud.point_seconds(arr,10,{'mode':'relative','unit':'s','field':'time'},.1)

def test_invalid_layout():
 m=cloud.pack(np.ones((8,3)),100,'sensor');m.row_step=2
 with pytest.raises(ValueError,match='layout'):cloud.fields(m)

def test_cloud_units_offsets_and_filter(config):
 xyz=np.array([[2000.,j*100,800] for j in range(10)]+[[np.nan,0,0],[100000,0,0]])
 m=cloud.pack(xyz,1700000000000000000,'source',times=np.linspace(.09,0,len(xyz)))
 config['units']['xyz']='mm';config['time_offsets_s']['lidar']=.025
 out=cloud.normalize(m,config);points,a=cloud.unpack(out)
 assert out.width==10;assert bagio.ns(out)==bagio.ns(m)+25000000
 assert np.all(np.diff(a['time'])>=0);np.testing.assert_allclose(points[:,0],2)

def test_extrinsic_validation():
 with pytest.raises(ValueError,match='unit norm'):transform([0,0,0,0,0,0,0])

def test_phase_override_does_not_mutate(config):
 config['phase_overrides']={'localization':{'topics':{'lidar':'/other'}}}
 out=phase_config(config,'localization');assert out['topics']['lidar']=='/other';assert config['topics']['lidar']=='/points'
