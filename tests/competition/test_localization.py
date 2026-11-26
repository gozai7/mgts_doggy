import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from competition.localization import Guard, PROFILES, export_results, mapping_backend, select
from competition import backends
from competition.beluga_map import free_cells, prepare


def pose(x=0., yaw=0., z=0.):
    return [x, 0., z, *Rotation.from_euler("z", yaw).as_quat()]


def test_profiles_require_real_odometry_and_gravity(config):
    assert select(config,"baseline")["localization"]["prediction"] == "constant_velocity"
    assert select(config,"ndt_kiss")["localization"]["prediction"] == "external_tf"
    with pytest.raises(ValueError,match="kinematic"):
        select(config,"beluga")
    with pytest.raises(ValueError,match="height"):
        select(config,"beluga_kiss")
    config["beluga"]={"initial_base_height_m":.5}
    config["odometry"]={"gravity_aligned":True,"initial_roll_pitch_rad":[.1,-.2]}
    assert select(config,"beluga_kiss")["localization"]["backend"] == "beluga"
    assert config["localization"]["prediction"] == "constant_velocity"


def test_external_odom_uses_pose_tf_not_empty_kiss_twist(config,tmp_path):
    c=select(config,"ndt_kiss")
    p=backends.localization_config(c,tmp_path/"params.yaml","map.pcd")
    assert p["enable_map_odom_tf"] and p["use_odom_tf_prediction"]
    assert not any(p[k] for k in ("use_odom","use_twist_prediction","predict_pose_from_previous_delta","use_imu_preintegration"))


def test_backend_alias_and_invalid(config):
    config["mapping"]["backend"]="rko_lio"
    assert mapping_backend(config)=="rko-lio"
    config["mapping"]["backend"]="unknown"
    with pytest.raises(ValueError):mapping_backend(config)


def test_fallback_is_anchored_and_does_not_reset_odometry():
    guard=Guard({"confirmations":2})
    n=10**9
    assert guard.update(n,pose(100),pose(10),True)[0]=="GLOBAL_SEARCH"
    assert guard.update(n+10**8,pose(101),pose(11),True)[0]=="TRACKING"
    state,source,out=guard.update(n+2*10**8,None,pose(12),False)
    assert (state,source)==("ODOMETRY_ONLY","odometry_only")
    assert out[0]==pytest.approx(102)
    assert guard.update(n+7*10**9,None,pose(15),False)[2] is None


def test_uninitialized_odometry_never_becomes_global():
    guard=Guard()
    assert guard.update(1,None,pose(50),False)[2] is None
    with pytest.raises(ValueError):guard.update(1,None,pose(50),False)


def test_implausible_correction_is_not_accepted():
    guard=Guard({"confirmations":1})
    guard.update(10**9,pose(0),pose(0),True)
    state,source,out=guard.update(11*10**8,pose(50),pose(.1),True)
    assert source=="odometry_only" and out[0]==pytest.approx(.1)


def test_implausible_odometry_does_not_jump_fallback():
    guard=Guard({"confirmations":1})
    guard.update(10**9,pose(),pose(),True)
    assert guard.update(11*10**8,None,pose(30),False)[2] is None
    assert guard.update(12*10**8,None,pose(30.1),False)[2] is None


def test_recovery_can_replace_old_anchor_after_consistent_new_hypothesis():
    guard=Guard({"confirmations":2})
    guard.update(10**9,pose(),pose(),True)
    guard.update(11*10**8,pose(.1),pose(.1),True)
    # A recovered globally consistent hypothesis can differ from the old anchor.
    assert guard.update(12*10**8,pose(10),pose(.2),True)[1]=="odometry_only"
    assert guard.update(13*10**8,pose(10.1),pose(.3),True)[1]=="odometry_only"
    assert guard.update(14*10**8,pose(10.2),pose(.4),True)[1]=="map_based"


def test_export_labels_every_scan_and_does_not_hide_failure(tmp_path):
    ns=[10**9+i*10**8 for i in range(5)]
    events=[{"stamp_ns":n,"accepted":i in (0,1,3,4)} for i,n in enumerate(ns)]
    poses={n:pose(i) for i,n in enumerate(ns) if i!=2}
    odo={n:pose(i+10) for i,n in enumerate(ns)}
    metrics=export_results(tmp_path,events,poses,odo,{"confirmations":2})
    assert metrics["confirmed_poses"]==2
    csv=(tmp_path/"trajectory.csv").read_text()
    assert "unconfirmed" in csv and "odometry_only" in csv
    assert len(csv.splitlines())==6
    assert metrics["absolute_accuracy"] is None


def test_gap_is_reported_as_missing_time(tmp_path):
    events=[{"stamp_ns":10**9,"accepted":True},{"stamp_ns":8*10**9,"accepted":False}]
    report=export_results(tmp_path,events,{10**9:pose()}, {},guard=False)
    assert report["time_by_source_s"]["missing"]==7


def test_offline_initialization_prefix_is_not_reported_as_instant_tracking(tmp_path):
    events=[{"stamp_ns":n*10**9,"accepted":True} for n in (1,2,3)]
    report=export_results(tmp_path,events,{e["stamp_ns"]:pose() for e in events},{},guard=False,availability_after_s=1.5)
    assert report["initialization_time_s"]==1.5
    assert report["confirmed_time_fraction"]==1.
    assert report["online_confirmed_time_fraction"]==.25


def occupancy(tmp_path):
    import yaml
    pixels=np.full((20,20),254,np.uint8)
    pixels[:2]=205; pixels[10,10]=0
    (tmp_path/"map.pgm").write_bytes(b"P5\n# comment\n20 20\n255\n"+pixels.tobytes())
    p=tmp_path/"map.yaml"
    p.write_text(yaml.safe_dump({"image":"map.pgm","resolution":.1,"origin":[4.,5.,np.pi/2],
                                "free_thresh":.196,"occupied_thresh":.65,"negate":0}))
    return p


def test_free_mask_preserves_rotated_origin_and_excludes_unknown(tmp_path):
    xy,res=free_cells(occupancy(tmp_path),.2)
    assert res==.1 and len(xy)>0
    assert xy[:,0].max()<4 and xy[:,1].min()>5
    assert not np.any(np.linalg.norm(xy-np.array([3.05,6.05]),axis=1)<.2)


def test_map_cache_checks_artifacts_and_content(tmp_path):
    occ=occupancy(tmp_path)
    pcd=tmp_path/"input.pcd"
    pcd.write_text("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 5\nDATA ascii\n"+"0 0 0\n"*5)
    converter=tmp_path/"converter.py"
    converter.write_text("import pathlib,sys\npathlib.Path(sys.argv[2]).write_bytes(b'test-vdb')\n")
    import sys
    cache=prepare(pcd,occ,tmp_path/"cache",{},[sys.executable,str(converter)])
    assert prepare(pcd,occ,tmp_path/"cache",{},["DO_NOT_RUN"])==cache
    (cache/"map.vdb").write_bytes(b"corrupt")
    with pytest.raises(ValueError,match="Corrupt"):
        prepare(pcd,occ,tmp_path/"cache",{},["DO_NOT_RUN"])


def test_spatial_deskew_keeps_sensor_lever_arm_and_refuses_extrapolation():
    from competition.trajectory import deskew_scan
    from competition.config import transform
    ts=np.array([1.,1.1,1.2])
    quats=Rotation.from_euler("z",[0.,.1,.2]).as_quat()
    odometry=np.column_stack([ts,np.zeros((3,3)),quats])
    mount=transform([1.,0.,0.,0.,0.,0.,1.])
    world=np.array([5.,2.,.3])
    raw=np.array([Rotation.from_quat(q).inv().apply(world)-mount[:3,3] for q in quats])
    result,valid=deskew_scan(raw,ts-1.2,1.2,odometry,mount)
    expected=Rotation.from_quat(quats[-1]).inv().apply(world)-mount[:3,3]
    np.testing.assert_allclose(result,np.tile(expected,(3,1)),atol=1e-12)
    assert valid.all()
    _,valid=deskew_scan(raw,np.array([-1.,-.1,0.]),1.2,odometry,mount)
    assert not valid[0]


def test_kiss_deskew_requires_a_verified_sweep_end(config):
    config["odometry"]={"deskew":True}
    with pytest.raises(ValueError,match="sweep end"):
        select(config,"ndt_kiss")
    config["point_time"]["header_reference"]="end"
    assert select(config,"ndt_kiss")["odometry"]["deskew"]


def test_degenerate_point_times_and_double_deskew_are_explicit(config):
    from competition import cloud
    points=np.array([[2.,i*.1,.5] for i in range(10)])
    config["odometry"]={"source":"kiss_icp"}
    message=cloud.pack(points,10**9,"lidar",times=np.zeros(10))
    with pytest.raises(ValueError,match="varying"):
        cloud.normalize(message,config)
    config["odometry"]["source"]="existing"
    config["lidar"]["already_deskewed"]=True
    with pytest.raises(ValueError,match="double deskew"):
        cloud.normalize(message,config)
    config["point_time"]["mode"]="none"
    assert "time" not in cloud.fields(cloud.normalize(message,config)).dtype.names


def test_paired_cloud_calibration_is_verified_not_guessed():
    from tools.infer_cloud_mount import fit
    rng=np.random.default_rng(9); raw=rng.normal(size=(200,3))
    fields=np.zeros(200,dtype=[("ring","u2"),("time","f8"),("intensity","f4")])
    fields["time"]=np.arange(200)*.001
    rotation=Rotation.from_euler("xyz",[.2,2.8,-.1]); translation=np.array([.28,0.,-.024])
    ids=rng.permutation(200)[:100]
    target=rotation.apply(raw[ids])+translation
    mount,error,count=fit((raw,fields),(target,fields[ids]))
    np.testing.assert_allclose(mount[:3],translation,atol=1e-12)
    assert (rotation.inv()*Rotation.from_quat(mount[3:])).magnitude()<1e-12
    assert count==100 and error<1e-12
    target[0,0]+=.01
    with pytest.raises(ValueError,match="rigid mount"):
        fit((raw,fields),(target,fields[ids]))


def test_synthetic_odometry_has_no_B_global_start_leak(tmp_path):
    from tools.synthetic_bags import generate
    from competition import bagio
    from competition.trajectory import read_tum
    generate(tmp_path/"fixture",duration=8.,with_odometry=True)
    truth=read_tum(tmp_path/"fixture/localization_truth.tum")
    assert np.linalg.norm(truth[0,1:4])>.1
    with bagio.reader(tmp_path/"fixture/localization") as reader:
        conn=next(c for c in reader.connections if c.topic=="/robot/odometry")
        _,_,raw=next(reader.messages(connections=[conn]))
        message=reader.deserialize(raw,conn.msgtype)
        np.testing.assert_allclose(bagio.pose_values(message.pose.pose),pose(),atol=1e-12)
        assert np.diag(message.pose.covariance.reshape(6,6)).min()>0
