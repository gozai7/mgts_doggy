import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from competition.trajectory import interpolate,change_body,metrics,save,read_tum
from competition.config import transform
from competition.occupancy import ray_cells,grid_from_rays,export_grid
from competition.evaluate import compare
from competition.timing import correlate
from competition import bagio

def trajectory():
 return np.array([[1,0,0,0,0,0,0,1],[2,1,0,0,0,0,1,0]],float)

def test_frame_lever_arm():
 a=trajectory();b=change_body(a,transform([1,0,0,0,0,0,1]))
 np.testing.assert_allclose(b[:,1:4],[[1,0,0],[0,0,0]],atol=1e-10)

def test_interpolation_gaps_slerp_no_extrapolation():
 a=trajectory();p,ok=interpolate(a,[.5,1,1.5,2,2.5],2)
 np.testing.assert_array_equal(ok,[False,True,True,True,False])
 assert p[2,0]==.5;assert Rotation.from_quat(p[2,3:]).magnitude()==pytest.approx(np.pi/2)
 _,ok=interpolate(a,[1,1.5,2],.5);np.testing.assert_array_equal(ok,[True,False,True])

def test_bad_tum(tmp_path):
 p=tmp_path/'t.tum';save(p,np.vstack([trajectory()[0]]*2))
 with pytest.raises(ValueError,match='increasing'):read_tum(p)

def test_input_duration_coverage():
 m=metrics(trajectory(),[0,4],gap_limit=2);assert m['coverage_fraction']==.25;assert m['absolute_accuracy'] is None

@pytest.mark.parametrize('end',[(3,0),(0,3),(-3,0),(0,-3),(3,3),(-3,-2)])
def test_bresenham_endpoint_never_cleared(end):
 cells=list(ray_cells((0,0),end));assert end not in cells;assert cells[0]==(0,0);assert len(cells)==max(map(abs,end))

def test_unknown_cells_and_occupied_endpoints(tmp_path):
 scans=[(np.array([0,0,0]),np.array([[2.,0.,.5]]))]*2
 grid,origin,report=grid_from_rays(scans,1.)
 def at(x,y):return grid[int(y-origin[1]),int(x-origin[0])]
 assert at(0,0)==0;assert at(2,0)==100;assert at(0,1)==-1
 export_grid(tmp_path/'map',grid,origin,1.,1000000000,'map')
 data=(tmp_path/'map/map.pgm').read_bytes();assert data.startswith(b'P5\n')
 with bagio.reader(tmp_path/'map/occupancy_bag') as r:
  c,t,raw=next(r.messages());m=r.deserialize(raw,c.msgtype)
  np.testing.assert_array_equal(m.data,grid.ravel());assert m.header.frame_id=='map'

def test_compare_requires_known_frame_transform(tmp_path):
 p=tmp_path/'a.tum';save(p,trajectory())
 with pytest.raises(ValueError,match='T_a_b'):compare(p,p)
 assert compare(p,p,[0,0,0,0,0,0,1],max_gap=2)['position_disagreement_rmse_m']==0

def test_offset_sign_and_low_excitation():
 t=np.arange(0,8,.005)
 def w(t):return np.column_stack([np.sin(t*3)+.3*np.sin(t*11),np.sin(t*2),np.cos(t*4)])
 offset=.035
 r=correlate(t,w(t+offset),t,w(t),.1,.001)
 assert r['offset_s']==pytest.approx(offset,abs=.001);assert not r['applied']
 r=correlate(t,np.ones((len(t),3)),t,np.ones((len(t),3)))
 assert r['offset_s'] is None and r['status']=='insufficient_excitation'
