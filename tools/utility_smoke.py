#!/usr/bin/env python3
"""ROS-free integration check. Known fixture poses validate export, not SLAM accuracy."""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.synthetic_bags import generate
from tools.export_plot_csv import export
from competition.config import load,write_json
from competition.doctor import audit
from competition.normalize import normalize_bag
from competition.occupancy import build
from competition.timing import estimate
from competition.perturb import perturb
from competition import backends

def run(output):
    started=time.monotonic();base=Path(output).resolve()
    c=load(generate(base))
    def hashes():
        return {str(p.relative_to(base)):hashlib.sha256(p.read_bytes()).hexdigest()
                for phase in ('mapping','localization') for p in (base/phase).rglob('*') if p.is_file()}
    before=hashes();report={'synthetic_fixture_only':True,'executed_slam':False}
    for phase in ('mapping','localization'):
        a=audit(base/phase,base/'audit'/phase)
        if a['errors']:raise RuntimeError(a['errors'])
        report[phase+'_doctor_messages']=a['read_messages']
        report[phase+'_counts']=normalize_bag(base/phase,base/'normalized'/phase,c)['counts']
    report['occupancy_fixture']=build(base/'normalized/mapping/bag',base/'mapping_truth.tum',
                                     base/'occupancy_with_fixture_poses_NOT_SLAM',c)
    report['csv_rows']=export(base/'normalized/mapping/bag',base/'diagnostics/plot.csv')
    offset=estimate(base/'normalized/mapping/bag',base/'mapping_truth.tum',c)
    report['time_offset']={k:offset[k] for k in ('status','offset_s','peak_correlation','applied')}
    perturb(base/'mapping',base/'fault_20ms',topic='/imu/data',header_shift_s=.02,seed=42)
    fault=audit(base/'fault_20ms/bag',base/'audit/fault_20ms')
    report['fault_header_record_s']=fault['topics']['/imu/data']['header_minus_record_s']
    if (backends.ROOT/'BACKEND').read_text().strip()=='glim':
        for mode in ('cpu','gpu'):
            cc=load(base/'dataset.yaml');cc['mapping']['mode']=mode
            backends.glim_config(cc,base/'prepared'/mode)
    else:
        backends.rko_configs(c,base/'prepared/rko',base/'normalized/mapping/bag',base/'rko_output')
    backends.localization_config(c,base/'prepared/localization.yaml',base/'placeholder_map.pcd')
    report['original_bags_unchanged']=before==hashes()
    if not report['original_bags_unchanged']:raise RuntimeError('Original bag changed')
    report['wall_time_s']=time.monotonic()-started
    write_json(base/'utility_smoke_report.json',report)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('new_output_directory');a=p.parse_args()
    print(json.dumps(run(a.new_output_directory),indent=2))
