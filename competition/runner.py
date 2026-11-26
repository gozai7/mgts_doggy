"""Common CLI. A successful command means required output files were validated."""
import argparse
import copy
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
import yaml
from .config import load, resolve, validate, write_json, fingerprint, phase_config, source_identity, sensor_contract, phase_fingerprint
from . import backends, bagio
from .normalize import normalize_bag
from .doctor import audit
from .trajectory import read_tum, metrics

ROOT = Path(__file__).resolve().parents[1]


def command(cmd, logfile, timeout):
    logfile = Path(logfile); logfile.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with logfile.open("w") as log:
        log.write("COMMAND " + json.dumps(cmd) + "\n"); log.flush()
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            rc = proc.wait(timeout=timeout)
        except BaseException:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL); proc.wait()
            raise
    receipt = {"command": cmd, "exit_code": rc, "wall_time_s": time.monotonic() - started,
               "peak_ram_bytes": None, "peak_vram_bytes": None}
    write_json(logfile.with_suffix(".json"), receipt)
    if rc:
        raise RuntimeError(f"Command failed ({rc}); see {logfile}")
    return receipt


def normalized(c, phase, root, baseline=False):
    dest = root / "normalized" / phase
    source = c["bags"]["mapping" if baseline else phase]
    effective = phase_config(c, "mapping" if baseline else phase)
    if baseline:
        effective["topics"]["imu"] = None
        effective["topics"]["odometry"] = None
    with bagio.reader(source) as r:
        resolved = resolve(effective, r, not baseline)
    if dest.exists():
        manifest = json.loads((dest / "manifest.json").read_text())
        if (not manifest["complete"] or manifest.get("sensor_contract_sha256") != fingerprint(sensor_contract(resolved))
            or manifest["source"] != str(source) or manifest.get("source_identity") != source_identity(source)):
            raise ValueError("Normalized checkpoint differs from current config/source; select a new output directory")
    else:
        normalize_bag(source, dest, resolved, not baseline)
    return dest / "bag", resolved


def run_phase(phase, c, root, backend):
    if shutil.which("ros2") is None and phase in ("mapping", "localization", "baseline"):
        raise RuntimeError("ROS 2 is unavailable. Run inside the built competition Docker image")
    folder = root / phase
    if (folder / "complete.json").exists():
        saved = json.loads((folder / "complete.json").read_text())
        if saved.get("phase_config_sha256") != phase_fingerprint(c, phase):
            raise ValueError("Completed phase uses a different config; choose a new output root")
        required = ["trajectory_optimized.tum", "map.pcd"] if phase == "mapping" else ["trajectory.tum", "diagnostics.json"]
        if not all((folder / x).is_file() and (folder / x).stat().st_size > 0 for x in required):
            raise RuntimeError("Completed phase has missing artifacts; use a new output root")
        normalized(c, phase, root, phase == "baseline")
        if phase == "localization" and saved.get("map_identity") != source_identity(root / "mapping/map.pcd"):
            raise ValueError("Map changed after localization; choose a new output root")
        print(f"Reusing validated {phase} checkpoint {folder}")
        return
    if folder.exists():
        raise FileExistsError(f"Incomplete output already exists: {folder}. Inspect it; choose a new run or move this phase aside")
    bag, resolved = normalized(c, phase, root, phase == "baseline")
    folder.mkdir(parents=True)
    (folder / "config_resolved.yaml").write_text(yaml.safe_dump(resolved))
    configs = folder / "config"; configs.mkdir()
    timeout = c["mapping"]["timeout_s"]
    started = time.monotonic()
    try:
        if phase == "mapping":
            if backend == "glim":
                backends.glim_config(resolved, configs)
                cmd = ["ros2","run","glim_ros","glim_rosbag",str(bag),"--ros-args","-p",f"config_path:={configs}",
                       "-p",f"dump_path:={folder / 'dump'}","-p","auto_quit:=true"]
                command(cmd, root / "logs/glim.log", timeout)
            else:
                backends.rko_configs(resolved, configs, bag, folder)
                command(["ros2","run","rko_lio","offline_node","--ros-args","--params-file",str(configs / "rko.yaml")],
                        root / "logs/rko_frontend.log", timeout)
                with bagio.reader(folder / "backend_input") as r:
                    counts = {x.topic: x.msgcount for x in r.connections}
                    if counts.get("/rko_lio/frame", 0) < 2 or counts.get("/rko_lio/odometry") != counts.get("/rko_lio/frame"):
                        raise RuntimeError(f"Unpaired/empty RKO backend input: {counts}")
                command(["ros2","run","graph_based_slam","graph_slam_offline_runner","--ros-args","--params-file",str(configs / "graph.yaml")],
                        root / "logs/rko_backend.log", timeout)
            backends.export_mapping(backend, folder, resolved)
            manifest = json.loads((bag.parent / "manifest.json").read_text())
            span = [v * 1e-9 for v in manifest["spans_ns"]["lidar"]]
            quality = metrics(read_tum(folder / "trajectory_optimized.tum"), span)
            write_json(folder / "coverage.json", quality)
            if quality["coverage_fraction"] < c["mapping"]["min_trajectory_coverage"]:
                raise RuntimeError("Mapping trajectory coverage below configured threshold; inspect partial output")
        else:
            if phase == "localization":
                map_path = root / "mapping/map.pcd"
                if not (root / "mapping/complete.json").exists():
                    raise ValueError("Mapping has no successful checkpoint")
                loc = resolved["localization"]
                if loc["initial_pose"] is None and not loc.get("manual_initial_pose"):
                    from .cold_start import initialize
                    result = initialize(map_path, bag, resolved, root / "mapping/trajectory_optimized.tum")
                    write_json(folder / "initialization.json", result)
                    loc["initial_pose"] = result["initial_pose"]
                    (folder / "config_resolved.yaml").write_text(yaml.safe_dump(resolved))
                backends.localization_config(resolved, configs / "localization.yaml", map_path)
                params = configs / "localization.yaml"
            else:
                params = configs / "kiss.yaml"
                backends.yaml_params(params, {"use_sim_time": True, "base_frame": "", "lidar_odom_frame": "kiss_odom",
                    "publish_odom_tf": False, "publish_debug_clouds": True,
                    "data.deskew": resolved["point_time"]["mode"] != "none", "mapping.voxel_size": .25,
                    "data.min_range": float(resolved["lidar"]["range_m"][0]),
                    "data.max_range": float(resolved["lidar"]["range_m"][1])})
            cmd = [sys.executable,"-m","competition.ros_replay","--config",str(folder / "config_resolved.yaml"),
                   "--bag",str(bag),"--output",str(folder),"--params",str(params)]
            if phase == "baseline":
                cmd.append("--baseline")
            command(cmd, root / f"logs/{phase}.log", timeout)
            read_tum(folder / "trajectory.tum")
        write_json(folder / "complete.json", {"complete": True, "backend": backend, "phase": phase,
                   "phase_config_sha256": phase_fingerprint(c, phase),
                   "map_identity": source_identity(root / "mapping/map.pcd") if phase == "localization" else None,
                   "requested_config_sha256": fingerprint(c), "resolved_config_sha256": fingerprint(resolved),
                   "wall_time_s": time.monotonic() - started})
    except BaseException as e:
        write_json(folder / "failed.json", {"complete": False, "error": str(e), "wall_time_s": time.monotonic() - started})
        raise


def main(argv=None):
    p = argparse.ArgumentParser(description="Competition SLAM workflow")
    p.add_argument("action", choices=["doctor","normalize","prepare","mapping","localization","baseline","occupancy","evaluate","run-all"])
    p.add_argument("--config", default="configs/competition/dataset.yaml")
    p.add_argument("--phase", choices=["mapping","localization"], default="mapping")
    args = p.parse_args(argv); c = load(args.config)
    root = Path(c["output"]).resolve(); root.mkdir(parents=True, exist_ok=True)
    backend = (ROOT / "BACKEND").read_text().strip()
    if args.action == "doctor":
        result = audit(c["bags"][args.phase], root / "diagnostics" / args.phase)
        print(json.dumps({k: result[k] for k in ("bag","errors","warnings")}, ensure_ascii=False, indent=2))
        return 2 if result["errors"] else 0
    if args.action == "prepare":
        validate(c)
        d = root / "prepared"; d.mkdir(exist_ok=True)
        if backend == "glim":
            backends.glim_config(c, d / "mapping")
        else:
            backends.rko_configs(c, d / "mapping", root / "normalized/mapping/bag", root / "mapping")
        backends.localization_config(c, d / "localization.yaml", root / "mapping/map.pcd")
        print(f"Configuration generated in {d}; no SLAM was executed")
        return 0
    if args.action == "normalize":
        print(normalized(c, args.phase, root)[0]); return 0
    write_json(root / "diagnostics/system.json", {"python": platform.python_version(), "platform": platform.platform(),
               "backend": backend, "upstreams": json.loads((ROOT / "THIRD_PARTY.lock.json").read_text())})
    actions = ["mapping","localization","occupancy","evaluate"] if args.action == "run-all" else [args.action]
    if args.action == "run-all":
        for phase in ("mapping","localization"):
            report = audit(c["bags"][phase], root / "diagnostics" / phase)
            if report["errors"]:
                raise ValueError(f"Bag Doctor found errors in {phase}: {report['errors']}")
    for action in actions:
        if action in ("mapping","localization","baseline"):
            run_phase(action, c, root, backend)
        elif action == "occupancy":
            from .occupancy import build
            if not (root / "mapping/complete.json").exists():
                raise ValueError("Successful mapping checkpoint required")
            resolved = load(root / "mapping/config_resolved.yaml")
            resolved["occupancy"] = phase_config(c, "mapping")["occupancy"]
            validate(resolved)
            dest = root / "occupancy"
            marker = dest / "complete.json"
            identity = {"config_sha256": fingerprint(resolved),
                        "trajectory": source_identity(root / "mapping/trajectory_optimized.tum")}
            if marker.exists() and json.loads(marker.read_text()) == identity and all(
                    (dest / name).exists() for name in ("map.pgm", "map.yaml", "report.json", "occupancy_bag/metadata.yaml")):
                print(f"Reusing completed occupancy export {dest}")
                continue
            if dest.exists():
                raise FileExistsError(f"Incomplete/different occupancy export: {dest}; choose a new run or move it aside")
            use_backend_scans = backend == "rko-lio"
            bag = root / ("mapping/backend_input" if use_backend_scans else "normalized/mapping/bag")
            build(bag, root / "mapping/trajectory_optimized.tum", dest, resolved,
                  lidar_topic="/rko_lio/frame" if use_backend_scans else "/competition/lidar", deskewed=use_backend_scans)
            write_json(marker, identity)
        elif action == "evaluate":
            from .evaluate import evaluate
            evaluate(root)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, RuntimeError, FileNotFoundError, FileExistsError, TimeoutError) as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(2)
