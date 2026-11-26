import numpy as np
import pytest
from competition.evaluate import ground_truth
from competition.trajectory import save
from competition.pcd import xyz
from competition.backends import tune, rko_configs


def test_groundtruth_constant_origin_error_and_rigid_correction(tmp_path):
    t=np.arange(0,3,.1)
    ref=np.column_stack((t,t,np.zeros((len(t),4)),np.zeros(len(t)),np.ones(len(t))))
    est=ref.copy();est[:,1]+=2
    save(tmp_path/'ref.tum',ref);save(tmp_path/'est.tum',est)
    raw=ground_truth(tmp_path/'ref.tum',tmp_path/'est.tum',[0,0,0,0,0,0,1])
    assert raw['ape_translation_m']['rmse']==pytest.approx(2)
    assert raw['rpe_translation_m']['rmse']==pytest.approx(0,abs=1e-12)
    corrected=ground_truth(tmp_path/'ref.tum',tmp_path/'est.tum',[-2,0,0,0,0,0,1])
    assert corrected['ape_translation_m']['rmse']==pytest.approx(0,abs=1e-12)


def test_rpe_detects_motion_scale_error(tmp_path):
    t=np.arange(0,3,.1)
    ref=np.column_stack((t,t,np.zeros((len(t),5)),np.ones(len(t))))
    est=ref.copy();est[:,1]*=1.1
    save(tmp_path/'ref',ref);save(tmp_path/'est',est)
    r=ground_truth(tmp_path/'ref',tmp_path/'est',[0,0,0,0,0,0,1])
    assert r['rpe_translation_m']['rmse']==pytest.approx(.1,abs=1e-8)


@pytest.mark.parametrize('encoding',['ascii','binary'])
def test_pcd_fields_and_nonfinite_filter(tmp_path,encoding):
    a=np.array([[2,1,3,100],[5,4,6,200],[8,np.nan,9,300]],dtype='<f4')
    header=f'VERSION 0.7\nFIELDS y x z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 3\nDATA {encoding}\n'
    data=a.tobytes() if encoding=='binary' else '\n'.join(' '.join(map(str,row)) for row in a).encode()
    p=tmp_path/'map.pcd';p.write_bytes(header.encode()+data)
    np.testing.assert_allclose(xyz(p),[[1,2,3],[4,5,6]])


def test_unknown_tuning_and_gpu_rko_rejected(config,tmp_path):
    with pytest.raises(ValueError,match='Unsupported'):
        tune({'voxel_size':.2},{'voxxel_size':.3})
    config['mapping']['mode']='gpu'
    with pytest.raises(ValueError,match='cpu'):
        rko_configs(config,tmp_path,'input','output')
