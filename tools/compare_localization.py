#!/usr/bin/env python3
"""Draw labelled global trajectories without fitting transforms or filling gaps."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from competition.benchmark import comparison_report

if __name__=="__main__":
    parser=argparse.ArgumentParser(); parser.add_argument("results")
    args=parser.parse_args(); comparison_report(args.results)
