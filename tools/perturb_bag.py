#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition.perturb import perturb
p=argparse.ArgumentParser(); p.add_argument('source');p.add_argument('output');p.add_argument('--topic',required=True)
p.add_argument('--seed',type=int,default=42)
for flag,default in [('header-shift-s',0.),('record-shift-s',0.),('point-shift',0.),('point-scale',1.),('keep-points',1.),('drop-scans',0.),('gyro-noise',0.),('accel-noise',0.)]:
 p.add_argument('--'+flag,type=float,default=default)
p.add_argument('--gyro-clip',type=float);p.add_argument('--time-field',default='time');p.add_argument('--remove-time',action='store_true')
p.add_argument('--gap-s',nargs=2,type=float);p.add_argument('--gyro-bias',nargs=3,type=float,default=[0.,0.,0.])
p.add_argument('--tf-child');p.add_argument('--tf-translation',nargs=3,type=float,default=[0.,0.,0.])
p.add_argument('--tf-rpy-deg',nargs=3,type=float,default=[0.,0.,0.])
perturb(**vars(p.parse_args()))
