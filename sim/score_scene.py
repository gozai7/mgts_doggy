"""Score both runs in one frame fixed by the first available pose of mapping A."""

import argparse
from pathlib import Path

import numpy as np

from competition.config import pose7, transform, write_json
from competition.evaluate import ground_truth
from competition.trajectory import interpolate, read_tum


def score(reference, result):
    estimated = read_tum(result / "mapping/trajectory_optimized.tum")
    reference_a = read_tum(reference / "mapping.tum")
    anchor, valid = interpolate(reference_a, estimated[:1, 0])
    if not valid[0]:
        raise ValueError("First mapping pose falls outside the reference trajectory")
    world_from_map = pose7(transform(anchor[0]) @ np.linalg.inv(transform(estimated[0, 1:])))
    scores = {}
    for phase, filename in (
        ("mapping", "trajectory_optimized.tum"),
        ("localization", "trajectory.tum"),
    ):
        metrics = ground_truth(
            reference / f"{phase}.tum", result / phase / filename, world_from_map
        )
        scores[phase] = metrics
        print(
            f"{phase}: 3D APE RMSE {metrics['ape_translation_m']['rmse']:.3f} m"
            f" ({metrics['matched_ape']} matched poses)"
        )
    write_json(result / "diagnostics/ground_truth.json", scores)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    score(args.reference, args.result)
