"""Measure the NDT tracking stage at recorded Go2 speed for an explicit seed."""

import argparse
import json
from pathlib import Path
import os

import yaml
from competition import backends, ros_replay
from competition.config import load


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("config", "bag", "map", "seed", "output"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True)
    config = load(args.config)
    config["localization"]["initial_pose"] = json.loads(args.seed.read_text())["initial_pose"]
    params = args.output / "localization.yaml"
    backends.localization_config(config, params, args.map)
    resolved = args.output / "dataset.yaml"
    resolved.write_text(yaml.safe_dump(config))
    os.environ["MGTS_REPLAY_1X"] = "1"
    ros_replay.run(resolved, args.bag, args.output, params)
