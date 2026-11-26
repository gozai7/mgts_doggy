import copy
from competition.config import phase_fingerprint, sensor_contract, fingerprint
from competition.runner import normalized
from conftest import make_bag


def test_localization_pose_and_floor_do_not_invalidate_mapping(config,tmp_path):
    config['bags']={'mapping':str(make_bag(tmp_path/'source')), 'localization':'unused'}
    root=tmp_path/'results'
    bag,_=normalized(config,'mapping',root)
    mapping_hash=phase_fingerprint(config,'mapping')
    old_localization=phase_fingerprint(config,'localization')
    config['localization']['initial_pose'][0]=4.
    config['occupancy']['floor_z_m']=-.5
    assert phase_fingerprint(config,'mapping')==mapping_hash
    assert phase_fingerprint(config,'localization')!=old_localization
    assert normalized(config,'mapping',root)[0]==bag


def test_calibration_change_invalidates_mapping(config):
    before=phase_fingerprint(config,'mapping')
    config['extrinsics']['T_base_lidar'][0]+=.01
    assert phase_fingerprint(config,'mapping')!=before
