import json
from pathlib import Path
import pytest
import numpy as np
import yaml
from competition import backends
from competition.config import transform

@pytest.mark.parametrize('mode',['cpu','gpu'])
def test_glim_sensor_contract(config,tmp_path,mode):
 if not (backends.ROOT/'vendor/glim').exists():pytest.skip('GLIM source belongs to other archive')
 config['mapping']['mode']=mode
 backends.glim_config(config,tmp_path)
 sensors=json.loads((tmp_path/'config_sensors.json').read_text())['sensors']
 np.testing.assert_allclose(transform(sensors['T_lidar_imu']),np.linalg.inv(transform(config['extrinsics']['T_base_lidar']))@transform(config['extrinsics']['T_base_imu']))
 assert sensors['perpoint_relative_time'] and not sensors['autoconf_perpoint_times']
 ros=json.loads((tmp_path/'config_ros.json').read_text())['glim_ros'];assert ros['acc_scale']==1 and ros['imu_time_offset']==0
 gl=json.loads((tmp_path/'config.json').read_text())['global'];assert gl['config_odometry']==f'config_odometry_{mode}.json'
 assert 'libstandard_viewer.so' not in ros['extension_modules']

def test_rko_frame_order_and_full_backend(config,tmp_path):
 f,g=backends.rko_configs(config,tmp_path,'/data/bag',tmp_path/'out')
 assert f['extrinsic_lidar2base_quat_xyzw_xyz']==[0.,0.,0.,1.,.2,0.,0.]
 assert f['competition_backend_bag'].endswith('backend_input')
 assert g['offline_map_bundle'] and g['save_dense_pose_refined_map'] and g['loop_min_overlap_ratio']>=.6
 assert not g['use_scan_context'];assert not g['use_bev_descriptor']

def test_localizer_unknown_start_is_explicit(config,tmp_path):
 config['localization']['initial_pose']=None
 with pytest.raises(ValueError,match='initial_pose'):backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')
 config['localization']['manual_initial_pose']=True
 p=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd');assert not p['set_initial_pose']

@pytest.mark.parametrize('predictor',['constant_velocity','imu','kinematic'])
def test_only_one_motion_predictor(config,tmp_path,predictor):
 config['localization']['prediction']=predictor
 if predictor=='kinematic':config['topics']['odometry']='/odom'
 p=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')
 assert sum([p['use_odom'],p['use_imu_preintegration'],p['predict_pose_from_previous_delta']])==1
 assert p['initial_pose_qw']==1.;assert p['reject_above_score_threshold']

def test_config_parameters_are_in_pinned_localizer(config,tmp_path):
 source=backends.ROOT/'vendor/lidar_localization_ros2'
 if not source.exists():pytest.skip('vendor unavailable')
 text='\n'.join(p.read_text() for d in ['src','include'] for p in (source/d).rglob('*') if p.is_file() and p.suffix in ['.cpp','.hpp','.h'])
 params=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')
 assert all(k=='use_sim_time' or '"'+k+'"' in text for k in params)
