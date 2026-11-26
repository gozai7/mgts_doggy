"""Build A and localize B from the two read-only bag mounts selected by the user."""

import sys
from pathlib import Path

import yaml

from competition.config import load
from competition.runner import main as run_phase
from tools.go2_track import run as track


def main(name):
    if not name or name in {".", ".."} or Path(name).name != name:
        raise ValueError("RUN_NAME must be a single directory name")
    result = Path("/results") / name
    if result.exists():
        raise FileExistsError(f"{result} already exists; choose a new RUN_NAME")
    # Keep intermediate GLIM maps private to this A/B pair, even on repeat runs.
    work = Path("/results/.go2-work") / name
    work.mkdir(parents=True, exist_ok=False)
    config = load("/mgts/configs/competition/official-go2.yaml")
    config["bags"] = {"mapping": "/inputs/A.mcap", "localization": "/inputs/B.mcap"}
    config["output"] = str(work / "glim")
    path = work / "dataset.yaml"
    path.write_text(yaml.safe_dump(config))
    for phase in ("mapping", "route-odometry"):
        run_phase([phase, "--config", str(path)])
    track(config, result, work / "glim/mapping", window_se3=True)
    print(f"Results: results/{name}/ | RViz: make go2-result RUN_NAME={name}", flush=True)


if __name__ == "__main__":
    main(sys.argv[1])
