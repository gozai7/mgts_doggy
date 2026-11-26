#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition.config import load,write_json
from competition.timing import estimate
p=argparse.ArgumentParser();p.add_argument('normalized_bag');p.add_argument('independent_base_tum')
p.add_argument('--config',required=True);p.add_argument('--max-shift',type=float,default=.1);p.add_argument('--output',default='results/time_offset.json')
a=p.parse_args();r=estimate(a.normalized_bag,a.independent_base_tum,load(a.config),a.max_shift);write_json(a.output,r);print(r['status'],r['offset_s'])
