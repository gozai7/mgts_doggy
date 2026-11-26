#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition.doctor import audit
p=argparse.ArgumentParser(); p.add_argument('bag'); p.add_argument('--output', default='results/doctor')
p.add_argument('--max-messages', type=int, default=200000); p.add_argument('--cloud-samples', type=int, default=50)
a=p.parse_args(); r=audit(a.bag,a.output,a.max_messages,a.cloud_samples)
print(Path(a.output)/'bag_report.md')
sys.exit(2 if r['errors'] else 0)
