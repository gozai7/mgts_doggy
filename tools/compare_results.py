#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition.evaluate import compare
from competition.config import write_json
p=argparse.ArgumentParser();p.add_argument('trajectory_a');p.add_argument('trajectory_b');p.add_argument('--T-a-b',dest='transform',nargs=7,type=float,required=True)
p.add_argument('--output',default='results/comparison.json');a=p.parse_args()
r=compare(a.trajectory_a,a.trajectory_b,a.transform);write_json(a.output,r);print(r)
