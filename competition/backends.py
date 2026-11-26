"""Contracts checked against the commits in THIRD_PARTY.lock.json."""
from pathlib import Path
import json
import re
import shutil
import numpy as np
import yaml
from .config import transform, pose7, write_json
from .trajectory import read_tum, save, change_body

ROOT = Path(__file__).resolve().parents[1]


def jsonc(path):
    return json.loads(re.sub(r"/\*.*?\*/|//[^\n]*", "", Path(path).read_text(), flags=re.S))


def yaml_params(path, params, node="/**"):
    Path(path).write_text(yaml.safe_dump({node: {"ros__parameters": params}}, sort_keys=False))


def tune(params, overrides, allowed=None):
    """Reject misspelled or routing-related overrides before launching a backend."""
    allowed = set(params) if allowed is None else set(allowed)
    for key, value in overrides.items():
        if key not in allowed or key not in params:
            raise ValueError(f"Unsupported tuning parameter: {key}")
        expected = params[key]
        if isinstance(expected, bool):
            valid = isinstance(value, bool)
        elif isinstance(expected, (int, float)):
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)
        else:
            valid = isinstance(value, type(expected))
        if not valid:
            raise ValueError(f"Invalid value for tuning parameter {key}")
        params[key] = value


def glim_config(c, directory):
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    original = ROOT / "vendor" / "glim" / "config"
    for p in original.glob("*.json"):
        (directory / p.name).write_text(json.dumps(jsonc(p), indent=2))
    mode = c["mapping"]["mode"]
    profile = c["mapping"]["profile"]
    if mode not in ("cpu", "gpu"):
        raise ValueError("GLIM mode: cpu or gpu")
    if profile not in ("default_indoor", "sparse_lidar", "cpu_only", "gpu", "no_point_time"):
        raise ValueError("Unknown GLIM profile")
    if profile == "cpu_only" and mode != "cpu" or profile == "gpu" and mode != "gpu":
        raise ValueError("Profile and mapping.mode disagree")
    if profile == "no_point_time" and c["point_time"]["mode"] != "none":
        raise ValueError("no_point_time profile requires point_time.mode:none")
    glob = jsonc(directory / "config.json")
    for key in ("odometry", "sub_mapping", "global_mapping"):
        glob["global"]["config_" + key] = f"config_{key}_{mode}.json"
    write_json(directory / "config.json", glob)
    sensors = jsonc(directory / "config_sensors.json")
    sensors["sensors"].update({"T_lidar_imu": pose7(np.linalg.inv(transform(c["extrinsics"]["T_base_lidar"])) @ transform(c["extrinsics"]["T_base_imu"])),
        "global_shutter_lidar": c["point_time"]["mode"] == "none", "autoconf_perpoint_times": False,
        "autoconf_prefer_frame_time": True, "perpoint_relative_time": True, "perpoint_time_scale": 1.0})
    write_json(directory / "config_sensors.json", sensors)
    ros = jsonc(directory / "config_ros.json")
    ros["glim_ros"].update({"imu_topic": "/competition/imu", "points_topic": "/competition/lidar",
        "image_topic": "/competition/unused_image", "acc_scale": 1.0, "ang_scale": 1.0,
        "imu_time_offset": 0.0, "points_time_offset": 0.0,
        "imu_frame_id": c["frames"]["imu"], "lidar_frame_id": c["frames"]["lidar"],
        "base_frame_id": c["frames"]["imu"], "map_frame_id": c["frames"]["map"],
        "odom_frame_id": "glim_odom", "publish_imu2lidar": True,
        "extension_modules": ["libmemory_monitor.so", "librviz_viewer.so"]})
    # During offline GLIM replay no original TF is broadcast. Base export is performed explicitly.
    write_json(directory / "config_ros.json", ros)
    pre = jsonc(directory / "config_preprocess.json")
    pre["preprocess"].update({"distance_near_thresh": c["lidar"]["range_m"][0],
        "distance_far_thresh": c["lidar"]["range_m"][1], "downsample_resolution": .10,
        "random_downsample_target": 12000, "num_threads": 4})
    write_json(directory / "config_preprocess.json", pre)
    p = directory / f"config_odometry_{mode}.json"; odom = jsonc(p)
    if mode == "cpu":
        odom["odometry_estimation"].update({"registration_type": "GICP", "ivox_resolution": .5, "ivox_min_dist": .10, "num_threads": 4})
    else:
        odom["odometry_estimation"].update({"voxel_resolution": .2, "voxel_resolution_max": .25, "num_threads": 4})
    write_json(p, odom)
    p = directory / f"config_sub_mapping_{mode}.json"; sub = jsonc(p)
    sub["sub_mapping"].update({"keyframe_voxel_resolution": .25 if mode == "gpu" else .4,
        "submap_downsample_resolution": .15, "keyframe_update_min_points": 100 if profile == "sparse_lidar" else 300})
    write_json(p, sub)
    p = directory / f"config_global_mapping_{mode}.json"; glob = jsonc(p)
    glob["global_mapping"].update({"submap_voxel_resolution": .25 if mode == "gpu" else .4,
        "max_implicit_loop_distance": 5.0, "min_implicit_loop_overlap": .5})
    write_json(p, glob)
    files = {"preprocess": "config_preprocess.json", "odometry_estimation": f"config_odometry_{mode}.json",
             "sub_mapping": f"config_sub_mapping_{mode}.json", "global_mapping": f"config_global_mapping_{mode}.json"}
    for section, overrides in c["mapping"].get("glim_parameters", {}).items():
        if section not in files:
            raise ValueError(f"Unsupported GLIM tuning section: {section}")
        p = directory / files[section]; data = jsonc(p)
        tune(data[section], overrides, set(data[section]) - {"so_name"})
        write_json(p, data)


def rko_configs(c, directory, normalized, output):
    directory, output = Path(directory), Path(output)
    directory.mkdir(parents=True, exist_ok=True)
    if c["mapping"]["mode"] != "cpu":
        raise ValueError("This RKO-LIO integration supports mapping.mode: cpu")
    if c["mapping"]["profile"] not in ("default_indoor", "sparse_lidar", "no_point_time", "cpu_only"):
        raise ValueError("Unknown RKO profile (CPU only)")
    sparse = c["mapping"]["profile"] == "sparse_lidar"
    if c["mapping"]["profile"] == "no_point_time" and c["point_time"]["mode"] != "none":
        raise ValueError("no_point_time profile requires point_time.mode:none")
    to_qxyz = lambda a: list(map(float, a[3:] + a[:3]))
    extr = to_qxyz(c["extrinsics"]["T_base_lidar"])
    frontend = {"bag_path": str(normalized), "lidar_topic": "/competition/lidar", "imu_topic": "/competition/imu",
        "base_frame": c["frames"]["base"], "lidar_frame": c["frames"]["lidar"], "imu_frame": c["frames"]["imu"],
        "odom_frame": "rko_odom", "extrinsic_lidar2base_quat_xyzw_xyz": extr,
        "extrinsic_imu2base_quat_xyzw_xyz": to_qxyz(c["extrinsics"]["T_base_imu"]),
        "lidar_timestamps.force_relative": True, "lidar_timestamps.force_absolute": False,
        "deskew": c["point_time"]["mode"] != "none", "initialization_phase": True,
        "min_range": float(c["lidar"]["range_m"][0]), "max_range": float(c["lidar"]["range_m"][1]),
        "voxel_size": .4 if sparse else .25, "publish_deskewed_scan": True, "publish_local_map": False,
        "odom_topic": "/rko_lio/odometry", "deskewed_scan_topic": "/rko_lio/frame",
        "dump_results": True, "results_dir": str(output / "frontend"), "run_name": "mapping",
        "competition_backend_bag": str(output / "backend_input"), "output_subscriber_timeout_sec": 0.0}
    backend = {"bag_path": str(output / "backend_input"), "output_dir": str(output / "backend"),
        "registration_method": "GICP", "gicp_max_correspondence_distance": 1.0,
        "voxel_leaf_size": .25 if sparse else .15, "submap_distance_threshold": .5,
        "extrinsic_lidar2base_quat_xyzw_xyz": extr, "offline_map_bundle": True,
        "save_dense_pose_refined_map": True, "dense_pose_refined_scan_voxel_size": .10,
        "dense_pose_refined_output_voxel_size": .10,
        "threshold_loop_closure_score": .25, "loop_min_overlap_ratio": .65,
        "loop_max_translation_delta": 3.0, "loop_max_rotation_delta_deg": 30.0,
        "distance_loop_closure": 10.0, "range_of_searching_loop_closure": 3.0,
        "max_loop_candidate_count": 3, "num_adjacent_pose_cnstraints": 5,
        "loop_edge_robust_kernel_delta": 1.0, "use_scan_context": False,
        "use_bev_descriptor": False, "use_solid_descriptor": False, "use_triangle_descriptor": False,
        "use_gnss": False, "refine_map": False}
    tune(frontend, c["mapping"].get("rko_parameters", {}), {"voxel_size", "initialization_phase"})
    tune(backend, c["mapping"].get("graph_parameters", {}), set(backend) - {
        "bag_path", "output_dir", "extrinsic_lidar2base_quat_xyzw_xyz", "offline_map_bundle",
        "save_dense_pose_refined_map", "use_gnss", "refine_map"})
    yaml_params(directory / "rko.yaml", frontend)
    yaml_params(directory / "graph.yaml", backend, "graph_based_slam")
    return frontend, backend


def localization_config(c, path, map_path):
    l = c["localization"]; initial = l["initial_pose"]
    if initial is not None:
        transform(initial)
    predictor = l["prediction"]
    if predictor == "kinematic" and not c["topics"]["odometry"]:
        raise ValueError("Kinematic prediction requires an odometry topic")
    params = {"use_sim_time": True, "use_pcd_map": True, "map_path": str(map_path),
        "set_initial_pose": initial is not None, "global_frame_id": c["frames"]["map"],
        "base_frame_id": c["frames"]["base"], "odom_frame_id": c["frames"]["odom"],
        "registration_method": l["registration_method"], "ndt_resolution": float(l["ndt_resolution"]),
        "ndt_num_threads": 4, "score_threshold": float(l["score_threshold"]), "reject_above_score_threshold": True,
        "voxel_leaf_size": float(l["voxel_leaf_size"]), "scan_period": float(c["lidar"]["scan_period_s"]),
        "scan_min_range": float(c["lidar"]["range_m"][0]), "scan_max_range": float(c["lidar"]["range_m"][1]),
        "publish_tf": True, "enable_map_odom_tf": predictor == "kinematic",
        "use_odom": predictor == "kinematic", "use_odom_tf_prediction": False,
        "use_twist_prediction": False, "use_imu_preintegration": predictor == "imu",
        "imu_preintegration_use_base_frame_transform": True, "imu_accel_scale": 1.0,
        "predict_pose_from_previous_delta": predictor == "constant_velocity",
        "use_continuous_time_deskew": c["point_time"]["mode"] != "none",
        "continuous_time_deskew_mode": "imu_pose_history" if predictor == "imu" else "relative_motion",
        "point_timestamp_unit": "seconds", "enable_timer_publishing": False,
        "publish_bridge_pose_when_lost": False, "cloud_queue_depth": 50,
        "enable_reinitialization_request_output": True, "enable_reinitialization_request_latch": True}
    if initial is not None:
        params.update({"initial_pose_" + k: float(v) for k, v in zip(("x","y","z","qx","qy","qz","qw"), initial)})
    yaml_params(path, params)
    return params


def export_mapping(backend, folder, c):
    folder = Path(folder)
    if backend == "glim":
        raw_path, optimized_path = folder / "dump/odom_lidar.txt", folder / "dump/traj_lidar.txt"
        source_map = folder / "dump/map.pcd"
        body = np.linalg.inv(transform(c["extrinsics"]["T_base_lidar"]))
    else:
        raw_path, optimized_path = folder / "backend/trajectory_raw_dense.tum", folder / "backend/trajectory_optimized_dense.tum"
        source_map = folder / "backend/map_dense_optimized.pcd"
        body = np.eye(4)
    for src, name in ((raw_path, "trajectory_raw.tum"), (optimized_path, "trajectory_optimized.tum")):
        a = read_tum(src)
        save(folder / name, change_body(a, body), csv=False)
    a = read_tum(folder / "trajectory_optimized.tum")
    save(folder / "trajectory.tum", a)
    save(folder / "trajectory_lidar.tum", change_body(a, transform(c["extrinsics"]["T_base_lidar"])), csv=False)
    if not source_map.exists() or source_map.stat().st_size < 100:
        raise RuntimeError(f"Missing optimized point cloud {source_map}")
    with source_map.open("rb") as f:
        hdr = f.read(2048)
    match = re.search(rb"POINTS\s+(\d+)", hdr)
    if match is None or int(match.group(1)) == 0:
        raise RuntimeError("Invalid or empty optimized PCD")
    shutil.copyfile(source_map, folder / "map.pcd")
    write_json(folder / "frames.json", {"trajectory_optimized": f"{c['frames']['map']} <- {c['frames']['base']}",
        "trajectory_raw": f"algorithm local odom <- {c['frames']['base']}",
        "trajectory_lidar": f"{c['frames']['map']} <- {c['frames']['lidar']}",
        "quaternion_order": "xyzw", "units": "seconds, metres", "map": "globally optimized"})
