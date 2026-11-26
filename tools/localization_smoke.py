#!/usr/bin/env python3
"""Actual mapper + four localizers, with noisy relative odometry and unknown B start."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.synthetic_bags import generate
from competition.config import load,transform,pose7,write_json
from competition.trajectory import read_tum,interpolate
from competition.evaluate import ground_truth
from competition.experiments import run
import numpy as np
import yaml


def smoke(output,backend):
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=False)
    config_path=generate(output/"input",with_odometry=True)
    c=load(config_path); c["mapping"]["backend"]=backend
    c["output"]=str(output/"algorithms")
    config_path.write_text(yaml.safe_dump(c))
    write_json(output/"smoke_plan.json",{"mapping_backend":backend,"B_start":"unknown", "min_confirmed_fraction":.5,
        "p95_position_error_gate_m":.5,"seeds":[42],"ground_truth_is_not_algorithm_output":True})
    for action in ("mapping","occupancy"):
        subprocess.run([sys.executable,"-m","competition.runner",action,"--config",str(config_path)],check=True)
    bundle=Path(c["output"])
    mapping=read_tum(bundle/"mapping/trajectory_optimized.tum")
    reference=read_tum(output/"input/mapping_truth.tum")
    ref_pose,valid=interpolate(reference,[mapping[0,0]])
    if not valid[0]:raise RuntimeError("Cannot establish a fixed map/GT transform from A")
    T_map_reference=transform(mapping[0,1:])@np.linalg.inv(transform(ref_pose[0]))
    T_reference_map=pose7(np.linalg.inv(T_map_reference))
    c["beluga"]["initial_base_height_m"]=float(np.median(mapping[:,3]))
    c["localization"]["map_bundle"]=str(bundle)
    config_path.write_text(yaml.safe_dump(c))
    results=[]
    for profile in ("baseline","beluga","ndt_kiss","beluga_kiss"):
        row={"profile":profile,"success":False}
        try:
            metrics=run(c,bundle,profile)
            accuracy=ground_truth(output/"input/localization_truth.tum",bundle/"profiles"/profile/"trajectory.tum",T_reference_map)
            row.update(metrics,ground_truth=accuracy)
            row["success"]=accuracy["ape_translation_m"]["p95"]<=.5
        except (RuntimeError,ValueError,TimeoutError,FileNotFoundError,subprocess.SubprocessError) as e:
            row["error"]=str(e)
        results.append(row)
        write_json(output/"smoke_results.json",{"mapping_backend":backend,"T_reference_map_from_A":T_reference_map,"profiles":results,
            "synthetic_integration_test_only":True})
    from competition.benchmark import comparison_report
    comparison_report(bundle)
    return all(r["success"] for r in results)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("output");p.add_argument("--backend",choices=["glim","rko_lio"],required=True)
    a=p.parse_args(); sys.exit(0 if smoke(a.output,a.backend) else 1)
