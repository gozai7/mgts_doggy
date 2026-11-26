"""Compare the two visits to the long wall in official A using identical raw returns."""

import argparse
from collections import defaultdict
from pathlib import Path
import json

import numpy as np
from scipy.spatial.transform import Rotation
from competition import bagio, cloud
from competition.config import transform
from competition.trajectory import read_tum, interpolate


def world_points(points, times, trajectory):
    lo = max(0, np.searchsorted(trajectory[:, 0], times.min()) - 1)
    hi = np.searchsorted(trajectory[:, 0], times.max()) + 1
    if hi <= lo:
        return None
    pose, valid = interpolate(trajectory[lo:hi], times)
    if not valid.all():
        return None
    return Rotation.from_quat(pose[:, 3:]).apply(points) + pose[:, :3]


def check(bag, reference, candidates, mount, shift):
    tracks = {"odometry": reference, **candidates}
    aligned = {}
    for name, a in tracks.items():
        b, ok = interpolate(reference, a[:1, 0])
        if not ok[0]:
            raise ValueError("First pose outside odometry reference")
        T = transform(b[0]) @ np.linalg.inv(transform(a[0, 1:]))
        R = Rotation.from_matrix(T[:3, :3])
        aligned[name] = np.column_stack(
            (a[:, 0], R.apply(a[:, 1:4]) + T[:3, 3], (R * Rotation.from_quat(a[:, 4:])).as_quat())
        )
    turn = reference[np.argmax(reference[:, 2]), 0]
    table = {name: defaultdict(list) for name in tracks}
    with bagio.reader(bag) as r:
        for c, _, raw in r.messages(
            connections=[c for c in r.connections if c.topic == "/utlidar/cloud"]
        ):
            m = r.deserialize(raw, c.msgtype)
            xyz, f = cloud.unpack(m)
            xyz = xyz @ mount[:3, :3].T + mount[:3, 3]
            ts = bagio.ns(m) * 1e-9 + f["time"].astype(float)
            w = world_points(xyz, ts, reference)
            if w is None:
                continue
            # Physical ROI in the onboard odom frame; deliberately broad across the wall.
            # Same return IDs, visit and cells are used for all reconstructed maps.
            keep = (
                (w[:, 0] > 3.25)
                & (w[:, 0] < 5.1)
                & (w[:, 1] > 2)
                & (w[:, 1] < 9)
                & (w[:, 2] > 0.20)
                & (w[:, 2] < 1.20)
            )
            if not keep.any():
                continue
            xyz, ts, w = xyz[keep], ts[keep], w[keep]
            pass_id = int(ts[0] > turn)
            cells = np.floor((w[:, 1] - 2) / 0.5).astype(int) * 4 + np.floor(
                (w[:, 2] - 0.20) / 0.25
            ).astype(int)
            for name, a in aligned.items():
                points = w if name == "odometry" else world_points(xyz, ts + shift, a)
                if points is None:
                    continue
                for cell in np.unique(cells):
                    table[name][pass_id, int(cell)].append(points[cells == cell, :2])
    # Restrict all models to cells with at least 20 returns from BOTH passes.
    bins = set.intersection(
        *[
            {
                i
                for p, i in table[n]
                if all(sum(len(v) for v in table[n].get((j, i), [])) >= 20 for j in [0, 1])
            }
            for n in tracks
        ]
    )
    ids = sorted(bins)
    if len(ids) < 3:
        raise ValueError("Insufficient repeated coverage of the long wall")
    report = {}
    for n, cells in table.items():
        visits = np.array(
            [[np.median(np.vstack(cells[j, i]), axis=0) for i in ids] for j in [0, 1]]
        )
        outbound = visits[0]
        normal = np.linalg.svd(outbound - outbound.mean(0), full_matrices=False)[2][-1]
        separation = visits[1] - visits[0]
        delta = abs(separation @ normal)
        report[n] = dict(
            cells=len(ids),
            median_normal_m=float(np.median(delta)),
            p90_normal_m=float(np.percentile(delta, 90)),
            max_normal_m=float(delta.max()),
            median_xy_m=float(np.median(np.linalg.norm(separation, axis=1))),
        )
    return report


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path)
    p.add_argument("reference", type=Path)
    p.add_argument("trajectory", nargs="+", type=Path)
    p.add_argument("--mount", nargs=7, required=True, type=float, metavar="XYZ_QXYZW")
    p.add_argument("--shift", type=float, default=0.0, help="candidate pose time offset in seconds")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    r = check(
        a.bag,
        read_tum(a.reference),
        {str(k): read_tum(k) for k in a.trajectory},
        transform(a.mount),
        a.shift,
    )
    a.output.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps(r, indent=2))
