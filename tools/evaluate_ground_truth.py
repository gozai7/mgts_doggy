#!/usr/bin/env python3
"""Only use an independently measured ground truth; kinematic odometry is not GT."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition.evaluate import ground_truth
from competition.config import write_json
p = argparse.ArgumentParser()
p.add_argument('reference'); p.add_argument('estimated')
p.add_argument('--T-reference-map', nargs=7, type=float, required=True)
p.add_argument('--delta-s', type=float, default=1.)
p.add_argument('--max-gap-s', type=float, default=.5)
p.add_argument('--output', required=True)
a = p.parse_args()
r = ground_truth(a.reference, a.estimated, a.T_reference_map, a.delta_s, a.max_gap_s)
write_json(a.output, r)
print(r)
