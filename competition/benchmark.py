"""Artifact-first profile comparison. Failed and unavailable scenarios remain visible."""
import copy
import json
import csv
import html
import subprocess
from pathlib import Path

from .config import write_json, fingerprint, source_identity
from .localization import PROFILES, sha256

DEFAULT_CRITERIA = {
    "initialization_success_position_m": .5,
    "initialization_success_angle_deg": 10.,
    "minimum_initialization_success_gain": .10,
    "minimum_relative_p95_improvement": .15,
    "maximum_confirmed_coverage_loss": .02,
    "coverage_metric": "online_confirmed_time_fraction",
    "maximum_localization_wall_time_s": 600.,
    "interpretation": "Preliminary engineering gates; success/false-init rates require independent GT and multiple trials.",
}


def benchmark(c, root):
    from .experiments import run, map_bundle
    from .perturb import perturb
    from .evaluate import ground_truth
    root=Path(root)
    settings=c.get("benchmark",{})
    bundle=map_bundle(c,root)
    criteria={**DEFAULT_CRITERIA,**settings.get("criteria",{})}
    plan={"criteria":criteria,"settings":settings,"map_sha256":sha256(bundle/"mapping/map.pcd"),
          "input_config":c,"B_identity":source_identity(c["bags"]["localization"])}
    plan_path=root/"benchmark_plan.json"
    if plan_path.exists() and json.loads(plan_path.read_text())!=plan:
        raise ValueError("Benchmark plan/map changed; use a new output directory")
    write_json(plan_path,plan)  # Written before any algorithm runs.
    rows=[]
    scenarios=settings.get("scenarios",[{"name":"normal"},{"name":"unknown_start","unknown_start":True},
        {"name":"lidar_50","perturb":{"keep_points":.5}},{"name":"lidar_25","perturb":{"keep_points":.25}},
        {"name":"scan_drops","perturb":{"drop_scans":.2}}])
    for scenario in scenarios:
        name=scenario["name"]
        if Path(name).name!=name or name in (".",".."):
            raise ValueError("Scenario names must be directory names")
        case=copy.deepcopy(c)
        case["localization"]["map_bundle"]=str(bundle)
        if scenario.get("unknown_start"):
            case["localization"]["initial_pose"]=None
            case["localization"]["manual_initial_pose"]=False
        if "initial_pose" in scenario:
            case["localization"]["initial_pose"]=scenario["initial_pose"]
            case["localization"]["manual_initial_pose"]=False
        if "bag" in scenario:
            case["bags"]["localization"]=scenario["bag"]
        if scenario.get("perturb"):
            output=root/"fault_bags"/name
            options=scenario["perturb"]
            from . import bagio
            from .config import resolve, phase_config
            with bagio.reader(case["bags"]["localization"]) as reader:
                effective=resolve(phase_config(case,"localization"),reader,require_imu=False)
            topic=effective["topics"]["imu" if scenario.get("sensor")=="imu" else "lidar"]
            if not output.exists():
                output.parent.mkdir(parents=True,exist_ok=True)
                perturb(case["bags"]["localization"],output,topic=topic,seed=settings.get("fault_seed",42),**options)
            manifest=json.loads((output/"manifest.json").read_text())
            # Exact settings are stored by perturb(); never edit the original recording.
            case["bags"]["localization"]=str(output/"bag") if (output/"bag").exists() else str(output)
        for profile in settings.get("profiles",list(PROFILES)):
            if profile not in PROFILES: raise ValueError(f"Unknown profile: {profile}")
            seeds=settings.get("seeds",[42,43,44]) if profile.startswith("beluga") else [42]
            for seed in seeds:
                trial=copy.deepcopy(case)
                trial.setdefault("beluga",{})["seed"]=seed
                run_name=f"{profile}-seed{seed}"
                trial["localization"]["run_name"]=run_name
                case_root=root/"scenarios"/name
                case_root.mkdir(parents=True,exist_ok=True)
                row={"scenario":name,"profile":profile,"seed":seed,"success":False}
                try:
                    metrics=run(trial,case_root,profile)
                    row.update(metrics,success=True)
                    if settings.get("ground_truth"):
                        if "T_reference_map" not in settings:
                            raise ValueError("Provide one T_reference_map from A/calibration for every profile; no B fitting")
                        row["ground_truth_metrics"]=ground_truth(settings["ground_truth"],case_root/"profiles"/run_name/"trajectory.tum",settings["T_reference_map"])
                except (ValueError,RuntimeError,FileNotFoundError,FileExistsError,TimeoutError,subprocess.SubprocessError) as e:
                    partial=case_root/"profiles"/run_name/"metrics.json"
                    if partial.exists():row.update(json.loads(partial.read_text()))
                    row["success"]=False
                    row["error"]=str(e)
                rows.append(row)
                write_json(root/"comparison.json",{"plan":plan,"results":rows})
    lines=["# Localization comparison","", "All profiles use one unchanged map. Missing GT means absolute accuracy is unknown.","",
           "| Scenario | Profile | Seed | Completed | Confirmed time | Initialization (s) | Wall time (s) |", "|---|---|---:|---|---:|---:|---:|"]
    for r in rows:
        def number(key):
            return f"{r[key]:.3f}" if isinstance(r.get(key),(int,float)) else "—"
        lines.append(f"| {r['scenario']} | {r['profile']} | {r['seed']} | {r['success']} | {number('confirmed_time_fraction')} | {number('initialization_time_s')} | {number('wall_time_s')} |")
    lines.extend(["","No automatic recommendation is made from missing GT or failed experiments."])
    (root/"comparison.md").write_text("\n".join(lines)+"\n")
    comparison_report(root)
    return rows


def comparison_report(root):
    """Standalone SVG overview; coordinate frames are never fitted to each other."""
    root=Path(root)
    trials=[]
    for path in sorted(root.rglob("profiles/*/metrics.json")):
        metrics=json.loads(path.read_text())
        if not (path.parent/"trajectory.csv").exists():continue
        with (path.parent/"trajectory.csv").open() as f:
            samples=list(csv.DictReader(f))
        trials.append((path,metrics,samples))
    groups={m.get("map_sha256") for _,m,_ in trials}
    if len(groups)>1:
        raise ValueError("Report contains different maps; generate one comparison per immutable map")
    points=[(float(r["x"]),float(r["y"])) for _,_,rows in trials for r in rows if r["x"]]
    if not points:
        (root/"trajectories.svg").write_text('<svg xmlns="http://www.w3.org/2000/svg" width="800" height="100"><text x="20" y="50">No confirmed global poses</text></svg>')
        return
    import numpy as np
    low=np.min(points,axis=0); high=np.max(points,axis=0)
    scale=min(700/max(high[0]-low[0],1),500/max(high[1]-low[1],1))
    colors=["#1565c0","#2e7d32","#8e24aa","#c62828","#00838f","#6d4c41"]
    height=max(650,80+18*len(trials))
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="{height}" viewBox="0 0 1100 {height}">',
         f'<rect width="1100" height="{height}" fill="white"/>','<text x="30" y="30">Global XY trajectories; dashed = odometry only; gaps preserved</text>']
    for index,(path,metrics,rows) in enumerate(trials):
        color=colors[index%len(colors)]
        label=str(path.parent.relative_to(root))
        svg.append(f'<text x="770" y="{60+18*index}" font-size="11" fill="{color}">{html.escape(label)}</text>')
        last=None
        for r in rows:
            if not r["x"]:
                last=None;continue
            xy=(30+(float(r["x"])-low[0])*scale,590-(float(r["y"])-low[1])*scale)
            if last is not None and float(r["t"])-last[1]<=1.:
                dash=' stroke-dasharray="5 4"' if r["source"]=="odometry_only" or last[2]=="odometry_only" else ''
                svg.append(f'<line x1="{last[0][0]:.2f}" y1="{last[0][1]:.2f}" x2="{xy[0]:.2f}" y2="{xy[1]:.2f}" stroke="{color}" stroke-width="1.5"{dash}/>')
            last=(xy,float(r["t"]),r["source"])
    svg.append(f'<text x="30" y="630">Map axes in metres; x=[{low[0]:.2f}, {high[0]:.2f}], y=[{low[1]:.2f}, {high[1]:.2f}]. No per-profile alignment.</text></svg>')
    (root/"trajectories.svg").write_text("\n".join(svg))
