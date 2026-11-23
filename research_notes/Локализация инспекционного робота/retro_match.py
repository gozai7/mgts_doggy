"""Проверка двух направлений локального scan-to-map на сохранённом office-B.

Пример запуска и ограничения описаны в маршрут.md. Для доступа к geometry и
evaluate укажите PYTHONPATH=/mgts/solver в образе mgts-solver.
"""

import argparse
import json
from itertools import product
from pathlib import Path
from time import perf_counter

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates

from evaluate import interpolate, recording_window, score, trajectory, transform
from geometry import compose, csv_save, inverse, load_amcl_map


def solve(map_yaml, route, output):
    config = load_amcl_map(map_yaml)
    grid = np.asarray(Image.open(map_yaml.parent / config["image"]).convert("L"))
    if config.get("mode") != "trinary" or config.get("negate") != 0:
        raise ValueError("experiment requires the project's trinary, negate=0 map")
    resolution = config["resolution"]
    origin_x, origin_y, _ = config["origin"]
    distance = distance_transform_edt(grid != 0) * resolution
    offsets = np.array(list(product(
        np.arange(-2, 3) * 0.05, np.arange(-2, 3) * 0.05,
        np.arange(-2, 3) * np.deg2rad(2),
    )))
    odometry = trajectory(route, "odometry.csv")
    amcl = trajectory(route)
    hits = np.load(route / "scan_hits.npy")
    hit_ns = np.rint(hits[:, 0] * 1e9).astype(np.int64)
    odom_ns = np.rint(odometry[:, 0] * 1e9).astype(np.int64)
    scans = {ns: hits[hit_ns == ns, 1:] for ns in np.unique(hit_ns)}
    if not set(scans).issubset(odom_ns):
        raise ValueError("scan preview and odometry lack matching stamps")
    if not np.allclose(amcl[[0, -1], 0], odometry[[0, -1], 0], rtol=0, atol=1e-8):
        raise ValueError("experiment assumes AMCL covers both odometry endpoints")

    def match(predicted, points):
        poses = predicted + offsets
        c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        x, y = points.T
        u = (poses[:, 0, None] + c * x - s * y - origin_x) / resolution - 0.5
        v = grid.shape[0] - 0.5 - (poses[:, 1, None] + s * x + c * y - origin_y) / resolution
        residuals = map_coordinates(distance, [v, u], order=1, mode="constant", cval=0.30)
        losses = np.mean(np.minimum(residuals, 0.30) ** 2, axis=1)
        return poses[np.argmin(losses)]

    output.mkdir(parents=True, exist_ok=False)
    last_correction = compose(amcl[-1, 1:], inverse(odometry[-1, 1:]))
    csv_save(output / "backward_odom.csv", transform(odometry, last_correction))
    for name, start, order in [
        ("forward", 0, range(len(odometry))),
        ("backward", -1, reversed(range(len(odometry)))),
    ]:
        began = perf_counter()
        correction = compose(amcl[start, 1:], inverse(odometry[start, 1:]))
        estimates = np.empty_like(odometry)
        for i in order:
            t, *odom = odometry[i]
            pose = compose(correction, odom)
            points = scans.get(odom_ns[i])
            if points is not None:
                pose = match(pose, points)
                correction = compose(pose, inverse(odom))
            estimates[i] = [t, *pose]
        print(f"{name} {len(estimates)} poses, {len(scans)} scans, {perf_counter()-began:.3f} s")
        csv_save(output / f"{name}.csv", estimates)


def evaluate(mapping, route, result, truth_a, truth_b):
    first = trajectory(mapping)[0]
    truth_map = trajectory(truth_a)
    if not truth_map[0, 0] <= first[0] <= truth_map[-1, 0]:
        raise ValueError("truth A does not cover the first map pose")
    world_to_map = compose(first[1:], inverse(interpolate(truth_map, [first[0]])[0]))
    window = json.loads((route / "run.json").read_text())["scan_window"]
    reference = recording_window(transform(trajectory(truth_b), world_to_map), window)
    samples = {"AMCL": trajectory(route)}
    for name in ["backward_odom", "forward", "backward"]:
        samples[name] = trajectory(result, f"{name}.csv")
    duration = window[1] - window[0]
    cuts = np.r_[np.arange(0, duration, 10), duration]
    for name, estimates in samples.items():
        print(name)
        for lo, hi in [(0, duration), *zip(cuts[:-1], cuts[1:])]:
            section = recording_window(reference, (window[0] + lo, window[0] + hi))
            print(f"  [{lo:g}, {hi:g}]", json.dumps(score(estimates, section, 1.0)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("solve")
    for label in ["map_yaml", "route", "output"]:
        run.add_argument(label, type=Path)
    compare = sub.add_parser("evaluate")
    for label in ["mapping", "route", "result", "truth_a", "truth_b"]:
        compare.add_argument(label, type=Path)
    args = vars(parser.parse_args())
    (solve if args.pop("action") == "solve" else evaluate)(**args)
