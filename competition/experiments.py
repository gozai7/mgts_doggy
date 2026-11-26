"""Run all localizers against an immutable map bundle, reusing normalized B and KISS odometry."""
import copy
import csv
import json
from pathlib import Path
import shutil
import sys
import time
import platform
import os
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from . import bagio, backends
from .config import load, resolve, sensor_contract, fingerprint, write_json, pose7, transform, phase_config, source_identity
from .localization import select, export_results, sha256
from .normalize import normalize_bag
from .trajectory import interpolate


def read_poses(path):
    return {row["stamp_ns"]: row["pose"] for row in map(json.loads, Path(path).read_text().splitlines())}


def normalized_b(c, root):
    c = copy.deepcopy(c)
    require_imu = c["localization"]["prediction"] == "imu"
    if not require_imu:
        c["topics"]["imu"] = None
    with bagio.reader(c["bags"]["localization"]) as reader:
        resolved = resolve(c, reader, require_imu)
    destination = root/"normalized"/"experiments"/fingerprint(sensor_contract(resolved))[:16]
    identity = source_identity(c["bags"]["localization"])
    if destination.exists():
        manifest = json.loads((destination/"manifest.json").read_text())
        if not manifest["complete"] or manifest["source_identity"] != identity:
            raise ValueError("Normalized B checkpoint source changed; select a new output")
    else:
        normalize_bag(c["bags"]["localization"], destination, resolved, require_imu)
    return destination/"bag", resolved


def existing_odometry(bag, max_gap, dense=False):
    stamps, rows = [], []
    with bagio.reader(bag) as reader:
        for conn, _, raw in reader.messages():
            if conn.topic not in ("/competition/lidar", "/competition/odometry"):
                continue
            message = reader.deserialize(raw, conn.msgtype)
            n = bagio.ns(message)
            if conn.topic.endswith("lidar"):
                stamps.append(n)
            else:
                rows.append([n*1e-9, *bagio.pose_values(message.pose.pose)])
    if len(rows) < 2:
        return {}
    values, valid = interpolate(np.asarray(rows), np.asarray(stamps)*1e-9, max_gap)
    samples={n: p.tolist() for n,p,ok in zip(stamps, values, valid) if ok}
    if dense:
        with bagio.reader(bag) as reader:
            for conn,_,raw in reader.messages(connections=[c for c in reader.connections if c.topic=="/competition/odometry"]):
                message=reader.deserialize(raw,conn.msgtype)
                samples.setdefault(bagio.ns(message),bagio.pose_values(message.pose.pose))
    return samples


def kiss_odometry(c, bag, root):
    from .runner import command
    settings = c.get("odometry", {})
    contract = {"source": source_identity(bag), "sensors": sensor_contract(c),
                "voxel_size_m": settings.get("voxel_size_m", .25),
                "deskew": settings.get("deskew",False),
                "position_variance_m2": settings.get("position_variance_m2", .1),
                "orientation_variance_rad2": settings.get("orientation_variance_rad2", .05),
                "kiss_commit": "1ffa7d7512f10bfc8b1185095011fa31184019e3"}
    folder = root/"odometry"/fingerprint(contract)[:16]
    cache_hit=(folder/"complete.json").exists()
    if (folder/"complete.json").exists():
        receipt = json.loads((folder/"complete.json").read_text())
        if receipt["contract"] != contract or receipt["poses_sha256"] != sha256(folder/"poses.jsonl"):
            raise ValueError("KISS odometry cache changed")
    else:
        if folder.exists():
            raise FileExistsError(f"Incomplete KISS run: {folder}; inspect it before retrying")
        folder.mkdir(parents=True)
        cfg = copy.deepcopy(c)
        (folder/"resolved_config.yaml").write_text(yaml.safe_dump(cfg))
        backends.yaml_params(folder/"kiss.yaml", {"use_sim_time": True, "base_frame": "", "lidar_odom_frame": "kiss_odom",
            "publish_odom_tf": False, "publish_debug_clouds": False,
            "data.deskew": bool(contract["deskew"]), "mapping.voxel_size": float(contract["voxel_size_m"]),
            "data.min_range": float(c["lidar"]["range_m"][0]), "data.max_range": float(c["lidar"]["range_m"][1]),
            "position_covariance": float(contract["position_variance_m2"]), "orientation_covariance": float(contract["orientation_variance_rad2"])})
        command([sys.executable,"-m","competition.ros_replay","--baseline","--config",str(folder/"resolved_config.yaml"),
                 "--bag",str(bag),"--output",str(folder),"--params",str(folder/"kiss.yaml")],folder/"replay.log",c["mapping"]["timeout_s"])
        write_json(folder/"complete.json", {"contract": contract, "poses_sha256": sha256(folder/"poses.jsonl")})
    poses = read_poses(folder/"poses.jsonl")
    tilt = settings.get("initial_roll_pitch_rad")
    if tilt is not None:
        # Align the initial BASE axes (not the LiDAR axes) with gravity without changing relative motion.
        initial = transform(next(iter(poses.values())))
        alignment = np.eye(4)
        alignment[:3,:3] = Rotation.from_euler("xyz", [*tilt, 0.]).as_matrix() @ initial[:3,:3].T
        alignment[:3,3] = -alignment[:3,:3] @ initial[:3,3]
        poses = {n: pose7(alignment@transform(p)) for n,p in poses.items()}
    return poses, folder, cache_hit


def map_bundle(c, root):
    bundle = Path(c["localization"].get("map_bundle") or root).resolve()
    if not (bundle/"mapping/complete.json").exists():
        raise ValueError(f"Successful mapping checkpoint required in {bundle}")
    if not (bundle/"mapping/map.pcd").is_file():
        raise ValueError("Map bundle has no optimized map.pcd")
    # A localization process must not change the map or mapping trajectory.
    return bundle


def mapping_reference(bundle):
    optimized=bundle/"mapping/trajectory_optimized.tum"
    return optimized if optimized.exists() else bundle/"mapping/trajectory.tum"


def failed_outputs(folder,bag,odometry,settings,profile,contract,error,elapsed,stage):
    """Retain every received stamp even if bootstrap/node exits before a normal export."""
    if (folder/"metrics.json").exists():return
    partial={}
    if (folder/"scan_events.jsonl").exists():
        partial={e["stamp_ns"]:e for e in map(json.loads,(folder/"scan_events.jsonl").read_text().splitlines())}
    events=[]
    with bagio.reader(bag) as reader:
        for conn,_,raw in reader.messages(connections=[c for c in reader.connections if c.topic=="/competition/lidar"]):
            n=bagio.ns(reader.deserialize(raw,conn.msgtype))
            events.append(partial.get(n,{"stamp_ns":n,"t":n*1e-9,"accepted":False,"state":"no_processing_result"}))
    if stage in ("configuration","map_preparation"):
        for event in events:
            if event["state"]=="no_processing_result":event["state"]="UNINITIALIZED"
    poses=read_poses(folder/"poses.jsonl") if (folder/"poses.jsonl").exists() else {}
    metrics=export_results(folder,events,poses,odometry,settings,guard=True)
    metrics.update(profile=profile,map_sha256=contract["map_sha256"],wall_time_s=elapsed,
                   failure_stage=stage,error=str(error),executed_localizer=(folder/"algorithm.log").exists())
    write_json(folder/"metrics.json",metrics)
    if not (folder/"resource_usage.json").exists():
        write_json(folder/"resource_usage.json",{"wall_time_s":elapsed,"peak_ram_bytes":None,"peak_vram_bytes":None,
            "note":"Failure before a complete sampled resource receipt"})


def beluga_params(c, bag, bundle, root, folder):
    from .beluga_map import prepare
    occupancy = bundle/"occupancy/map.yaml"
    if not occupancy.exists():
        raise ValueError("Beluga needs the map's free-space mask: run make occupancy on the mapping bundle first")
    settings = c.get("beluga", {})
    preexisting=set((root/"map_cache").iterdir()) if (root/"map_cache").exists() else set()
    started = time.monotonic()
    cache = prepare(bundle/"mapping/map.pcd", occupancy, root/"map_cache", settings)
    manifest = json.loads((cache/"manifest.json").read_text())
    initial = c["localization"]["initial_pose"]
    planar = [] if initial is None else [float(initial[0]),float(initial[1]),Rotation.from_quat(initial[3:]).as_euler("xyz")[2].item()]
    params = {"use_sim_time": True, "map_path": str(cache/"map.vdb"), "free_cells_path": str(cache/"free_cells.csv"),
              "T_base_lidar": list(map(float,c["extrinsics"]["T_base_lidar"])), "global_frame": c["frames"]["map"],
              "initial_base_height_m": float(settings["initial_base_height_m"]),
              "free_resolution_m": manifest["free_resolution_m"]}
    if planar:
        params["initial_pose_se2"] = planar
    for key, default in {"min_particles":2000,"max_particles":12000,"max_points":256,"seed":42,
                         "sigma_hit_m":.15,"motion_noise":.15,"max_xy_variance_m2":.16,
                         "max_yaw_variance_rad2":.03,"min_overlap":.65}.items():
        params[key] = settings.get(key,default)
    backends.yaml_params(folder/"config/localization.yaml",params)
    write_json(folder/"map_preparation.json", {"wall_time_s":time.monotonic()-started,"cache":str(cache),"cache_hit":cache in preexisting,"manifest":manifest})


def run(c, root, profile=None, odometry_only=False):
    from .runner import command
    total_started=time.monotonic()
    c = select(phase_config(c,"localization"), profile)
    root = Path(root).resolve()
    bag, resolved = normalized_b(c, root)
    source = resolved["odometry"]["source"]
    odometry_started=time.monotonic()
    if source == "kiss_icp":
        odometry, odometry_folder, odometry_cache_hit = kiss_odometry(resolved, bag, root)
    else:
        odometry = existing_odometry(bag, resolved["odometry"].get("max_gap_s", .5),dense=True)
        odometry_folder = None
        odometry_cache_hit = False
    odometry_wall_time=time.monotonic()-odometry_started
    if odometry_only:
        if odometry_folder is None:
            raise ValueError("odometry action currently runs KISS; choose --profile ndt_kiss")
        return {"odometry":str(odometry_folder)}
    bundle = map_bundle(resolved, root)
    profile = resolved["localization"]["profile"]
    # Different seeds/known-start experiments never overwrite another profile's result.
    run_name = resolved["localization"].get("run_name", profile)
    if Path(run_name).name != run_name or run_name in (".",".."):
        raise ValueError("localization.run_name must be a directory name")
    folder = root/"profiles"/run_name
    contract = {"config":c,"map_sha256":sha256(bundle/"mapping/map.pcd"),
                "B_identity":source_identity(c["bags"]["localization"])}
    if (folder/"complete.json").exists():
        receipt = json.loads((folder/"complete.json").read_text())
        if receipt["contract"] != contract or any(sha256(folder/k)!=v for k,v in receipt["artifacts"].items()):
            raise ValueError("Profile checkpoint changed; choose a new localization.run_name")
        return json.loads((folder/"metrics.json").read_text())
    if folder.exists():
        raise FileExistsError(f"Profile output already exists: {folder}; choose a new run_name")
    folder.mkdir(parents=True); (folder/"config").mkdir(); (folder/"logs").mkdir()
    started = time.monotonic()
    stage="configuration"
    try:
        write_json(folder/"system.json",{"python":platform.python_version(),"platform":platform.platform(),
            "ros_distro":os.environ.get("ROS_DISTRO"),"upstream_sources":json.loads((backends.ROOT/"THIRD_PARTY.lock.json").read_text()),
            "sources_are_not_proof_all_mappers_are_installed":True})
        if shutil.which("dpkg-query"):
            versions=subprocess.check_output(["dpkg-query","-W"],text=True)
            (folder/"runtime_apt_versions.txt").write_text(versions)
        loc = resolved["localization"]
        loc["supervised_tf"] = loc["prediction"] == "external_tf"
        loc["allow_failed_run_export"] = True
        (folder/"resolved_config.yaml").write_text(yaml.safe_dump(resolved))
        initialization_observation_s=0.
        if loc["prediction"] == "external_tf":
            if not odometry:
                raise ValueError("No odometry overlaps LiDAR timestamps")
            with (folder/"odometry.jsonl").open("w") as f:
                for n,p in sorted(odometry.items()): f.write(json.dumps({"stamp_ns":n,"pose":p})+"\n")
            resolved["odometry"]["trajectory_jsonl"] = str(folder/"odometry.jsonl")
        if loc["backend"] == "beluga":
            stage="map_preparation"
            beluga_params(resolved, bag, bundle, root, folder)
        else:
            if loc["initial_pose"] is None and not loc.get("manual_initial_pose"):
                stage="global_initialization"
                from .cold_start import initialize
                initialization = initialize(bundle/"mapping/map.pcd", bag, resolved, mapping_reference(bundle),
                                            odometry=odometry if source=="kiss_icp" else None)
                initialization_observation_s=initialization["attempts"][-1]["elapsed_s"]
                write_json(folder/"initialization.json",initialization); loc["initial_pose"] = initialization["initial_pose"]
            backends.localization_config(resolved, folder/"config/localization.yaml",bundle/"mapping/map.pcd")
        (folder/"resolved_config.yaml").write_text(yaml.safe_dump(resolved))
        before_replay_wall=time.monotonic()-total_started
        stage="localization"
        command([sys.executable,"-m","competition.ros_replay","--config",str(folder/"resolved_config.yaml"),"--bag",str(bag),
                 "--output",str(folder),"--params",str(folder/"config/localization.yaml")],folder/"logs/replay.log",c["mapping"]["timeout_s"])
        diagnostics = json.loads((folder/"diagnostics.json").read_text())
        resource=json.loads((folder/"logs/replay.json").read_text())
        write_json(folder/"resource_usage.json",resource)
        if odometry_folder is not None:
            shutil.copyfile(odometry_folder/"replay.json",folder/"odometry_resource_usage.json")
        result = export_results(folder, diagnostics["events"], read_poses(folder/"poses.jsonl"), odometry,
                                resolved.get("recovery"), guard=profile!="baseline",availability_after_s=initialization_observation_s)
        tracking_events=[e for e in diagnostics["events"] if e.get("supervisor_state")=="TRACKING" or profile=="baseline" and e["accepted"]]
        result["initialization_wall_time_s"]=before_replay_wall+tracking_events[0]["wall_elapsed_s"] if tracking_events else None
        result["initialization_observation_time_s"]=initialization_observation_s
        result["offline_prefix_replay"]=initialization_observation_s>0
        diagnostics["odometry_only_time_s"]=result["time_by_source_s"]["odometry_only"]
        write_json(folder/"diagnostics.json",diagnostics)
        result.update({"profile":profile,"map_sha256":contract["map_sha256"],"wall_time_s":time.monotonic()-started,
                       "total_wall_time_s":time.monotonic()-total_started,"odometry_preparation_wall_time_s":odometry_wall_time,
                       "global_initialization":c["localization"]["initial_pose"] is None,
                       "state_dimension":"SE(2) with 3D observations; z/roll/pitch conditional on odometry" if loc["backend"]=="beluga" else "SE(3)",
                       "odometry_cache":str(odometry_folder) if odometry_folder else None})
        result["odometry_cache_hit"]=odometry_cache_hit
        result["map_provenance"]=json.loads((bundle/"mapping/complete.json").read_text()).get("backend","unknown")
        write_json(folder/"metrics.json",result)
        if loc["backend"] == "beluga":
            with (folder/"particle_diagnostics.csv").open("w") as f:
                keys=["stamp_ns","particles","overlap","variance_x","variance_y","variance_yaw","update_ms","reinitializations"]
                w=csv.DictWriter(f,keys,extrasaction="ignore"); w.writeheader()
                for e in diagnostics["events"]: w.writerow({"stamp_ns":e["stamp_ns"],**e.get("values",{})})
        else:
            keys=["stamp_ns","accepted","message",*sorted({k for e in diagnostics["events"] for k in e.get("values",{})})]
            with (folder/"registration_diagnostics.csv").open("w") as f:
                w=csv.DictWriter(f,keys,extrasaction="ignore"); w.writeheader()
                for e in diagnostics["events"]:
                    w.writerow({"stamp_ns":e["stamp_ns"],"accepted":e["accepted"],"message":e.get("message"),**e.get("values",{})})
        write_json(folder/"frames.json",{"global_frame":resolved["frames"]["map"],"body_frame":resolved["frames"]["base"],
            "relative_frame":resolved["frames"]["odom"],"state_dimension":result["state_dimension"],
            "odometry_origin_is_not_map_origin":True,"global_alignment_fitted_to_B":False})
        if sha256(bundle/"mapping/map.pcd") != contract["map_sha256"]:
            raise RuntimeError("Map changed during localization")
        if not result["confirmed_poses"]:
            raise RuntimeError("No confirmed map poses; diagnostics and missing/unconfirmed rows retained")
        if result["confirmed_poses"]/result["scans"] < resolved["localization"]["min_accepted_fraction"]:
            raise RuntimeError("Confirmed pose fraction below configured minimum; complete failed-run diagnostics retained")
        artifacts={k:sha256(folder/k) for k in ("trajectory.tum","trajectory.csv","metrics.json","localization_status.csv","resolved_config.yaml")}
        write_json(folder/"complete.json", {"contract":contract,"artifacts":artifacts})
        return result
    except BaseException as e:
        write_json(folder/"failed.json", {"error":str(e),"wall_time_s":time.monotonic()-started})
        if isinstance(e,Exception):
            failed_outputs(folder,bag,odometry,resolved.get("recovery"),profile,contract,e,time.monotonic()-started,stage)
        raise
