import json
from pathlib import Path
import pytest
import numpy as np
import yaml
from competition import backends
from competition.config import transform
from competition.trajectory import read_tum

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


def test_fastlio_calibration_and_imu_trajectory_export(config, tmp_path):
    config["extrinsics"]["T_base_imu"] = [.1, .2, .3, 0., 0., 1., 0.]
    base_imu = transform(config["extrinsics"]["T_base_imu"])
    base_lidar = transform(config["extrinsics"]["T_base_lidar"])
    work = tmp_path / "backend"
    params = backends.fastlio_config(config, tmp_path / "config", "/data/bag", work)
    calibration = np.linalg.inv(base_imu) @ base_lidar
    np.testing.assert_allclose(params["mapping.extrinsic_T"], calibration[:3, 3])
    np.testing.assert_allclose(np.reshape(params["mapping.extrinsic_R"], (3, 3)), calibration[:3, :3])
    assert params["competition.point_time"]
    assert not params["mapping.extrinsic_est_en"]

    work.mkdir()
    (work / "trajectory_imu.tum").write_text("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n")
    (work / "map.pcd").write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n"
    )
    backends.export_mapping("fast-lio", tmp_path, config)
    base = read_tum(tmp_path / "trajectory.tum")
    lidar = read_tum(tmp_path / "trajectory_lidar.tum")
    np.testing.assert_allclose(transform(base[0, 1:]), np.linalg.inv(base_imu))
    np.testing.assert_allclose(transform(lidar[0, 1:]), calibration)
    np.testing.assert_array_equal(base, read_tum(tmp_path / "trajectory_raw.tum"))
    assert (tmp_path / "map.pcd").read_bytes() == (work / "map.pcd").read_bytes()

def test_localizer_unknown_start_is_explicit(config,tmp_path):
 config['localization']['initial_pose']=None
 assert not backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')['set_initial_pose']
 config['localization']['manual_initial_pose']=True
 p=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd');assert not p['set_initial_pose']

@pytest.mark.parametrize('predictor',['constant_velocity','imu','kinematic'])
def test_only_one_motion_predictor(config,tmp_path,predictor):
 config['localization']['prediction']=predictor
 if predictor=='kinematic':config['topics']['odometry']='/odom'
 p=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')
 assert sum([p['use_odom'],p['use_imu_preintegration'],p['predict_pose_from_previous_delta']])==1
 assert p['use_odom_tf_prediction'] == (predictor == 'kinematic')
 assert p['initial_pose_qw']==1.;assert p['reject_above_score_threshold']

def test_config_parameters_are_in_pinned_localizer(config,tmp_path):
 source=backends.ROOT/'vendor/lidar_localization_ros2'
 if not source.exists():pytest.skip('vendor unavailable')
 text='\n'.join(p.read_text() for d in ['src','include'] for p in (source/d).rglob('*') if p.is_file() and p.suffix in ['.cpp','.hpp','.h'])
 params=backends.localization_config(config,tmp_path/'l.yaml','/map.pcd')
 assert all(k=='use_sim_time' or '"'+k+'"' in text for k in params)
