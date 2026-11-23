"""Rank possible starts in final-A using only robot-frame lidar endpoints and relative odometry."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


def rotate(points, angle):
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


def angle_delta(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def load_scans(folder):
    odom = np.loadtxt(folder / "odometry.csv", delimiter=",", skiprows=1)
    hits = np.load(folder / "scan_hits.npy")
    stamps = np.unique(hits[:, 0])
    poses = np.column_stack([np.interp(stamps, odom[:, 0], col) for col in
                             (odom[:, 1], odom[:, 2], np.unwrap(odom[:, 3]))])
    assert np.isclose(stamps[0], odom[0, 0]) and stamps[-1] <= odom[-1, 0]
    poses[:, :2] = rotate(poses[:, :2] - poses[0, :2], -poses[0, 2])
    poses[:, 2] -= poses[0, 2]
    scans = [rotate(hits[hits[:, 0] == t, 1:], q[2]) + q[:2] for t, q in zip(stamps, poses)]
    return stamps - stamps[0], poses, scans


class Map:
    def __init__(self, path):
        cfg = yaml.safe_load(path.read_text())
        assert cfg["negate"] == 0 and cfg["mode"] == "trinary" and abs(cfg["origin"][2]) < 1e-9
        gray = np.flipud(np.asarray(Image.open(path.parent / cfg["image"]).convert("L")))
        self.origin, self.res = np.array(cfg["origin"][:2]), cfg["resolution"]
        prob = (255.0 - gray) / 255.0
        self.free = prob < cfg["free_thresh"]
        occupied = prob > cfg["occupied_thresh"]
        distances = distance_transform_edt(~occupied) * self.res
        known = self.free | occupied
        self.coarse = known * np.exp(-0.5 * (distances / 0.35) ** 2)
        self.fine = known * np.exp(-0.5 * (distances / 0.15) ** 2)

    def lookup(self, field, x, y):
        cols = np.floor((x - self.origin[0]) / self.res).astype(int)
        rows = np.floor((y - self.origin[1]) / self.res).astype(int)
        h, w = field.shape
        valid = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        return valid * field[rows.clip(0, h-1), cols.clip(0, w-1)]

    def scores(self, points, translations, angle, field):
        local = rotate(points, angle)
        x = translations[:, :1] + local[None, :, 0]
        y = translations[:, 1:] + local[None, :, 1]
        return self.lookup(field, x, y).mean(axis=1) * self.lookup(self.free, *translations.T)


def suppress(poses, scores, count):
    keep = []
    for i in np.argsort(-scores, kind="stable"):
        p = poses[i]
        if all(np.linalg.norm(p[:2] - poses[j, :2]) >= 0.75 or
               abs(angle_delta(p[2], poses[j, 2])) >= np.deg2rad(25) for j in keep):
            keep.append(i)
            if len(keep) == count:
                break
    return keep


def coarse_search(grid, scan):
    # All free samples on the 4-cell lattice, rotations through a full turn.
    rows, cols = np.nonzero(grid.free[::4, ::4])
    xy = grid.origin + (4 * np.column_stack((cols, rows)) + 0.5) * grid.res
    poses, scores = [], []
    for angle in np.deg2rad(np.arange(-180, 180, 8)):
        poses.append(np.column_stack((xy, np.full(len(xy), angle))))
        scores.append(grid.scores(scan, xy, angle, grid.coarse))
    poses, scores = np.vstack(poses), np.concatenate(scores)
    return poses[suppress(poses, scores, 24)]


def refine(grid, points, seed):
    pose = seed.copy()
    for xy_step, radius, deg_step, turns in [(0.10, 3, 2.0, 4), (0.035, 2, 0.5, 3)]:
        r = np.arange(-radius, radius + 1) * xy_step
        xy = pose[:2] + np.stack(np.meshgrid(r, r), axis=-1).reshape(-1, 2)
        best = -1
        for theta in pose[2] + np.deg2rad(np.arange(-turns, turns+1) * deg_step):
            scores = grid.scores(points, xy, theta, grid.fine)
            i = scores.argmax()
            if scores[i] > best:
                best = scores[i]
                winner = np.r_[xy[i], theta]
        pose = winner
    pose[2] = angle_delta(pose[2], 0)
    return pose, best


def rank(grid, seeds, train, validation):
    points = np.vstack(train)
    fits = [refine(grid, points, p) for p in seeds]
    poses = np.array([p for p, _ in fits])
    training = np.array([s for _, s in fits])
    if validation:
        scores = np.array([np.mean([grid.scores(s, p[None,:2], p[2], grid.fine)[0]
                                    for s in validation]) for p in poses])
    else:
        scores = training
    keep = suppress(poses, scores, 3)
    return [{"pose": poses[i].round(6).tolist(), "fit": round(training[i], 5),
             "score": round(scores[i], 5)} for i in keep]


def track_modes(grid, seeds, odom, scans, keys):
    # Extend each initial mode through motion. Alternate beams in each new scan for
    # fitting and scoring. Neighboring beams remain statistically dependent.
    modes = [refine(grid, scans[0], seed) for seed in seeds]
    by_origin = suppress(np.array([p for p, _ in modes]), np.array([s for _, s in modes]), 12)
    tracks = []
    for i in by_origin:
        origin, _ = modes[i]
        q = origin.copy()
        scores = []
        for prev, nxt in zip(keys, keys[1:]):
            delta = rotate(odom[nxt, :2] - odom[prev, :2], -odom[prev, 2])
            prior = np.r_[q[:2] + rotate(delta, q[2]), q[2] + odom[nxt, 2] - odom[prev, 2]]
            local = rotate(scans[nxt] - odom[nxt, :2], -odom[nxt, 2])
            q, _ = refine(grid, local[::2], prior)
            scores.append(grid.scores(local[1::2], q[None, :2], q[2], grid.fine)[0])
        tracks.append({"pose": origin.round(6).tolist(), "score": round(float(np.mean(scores)), 5),
                       "heldout_scores": np.round(scores, 5).tolist()})
    return sorted(tracks, key=lambda track: -track["score"])[:3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("map", type=Path)
    p.add_argument("inspection", type=Path)
    args = p.parse_args()
    start = time.perf_counter()
    grid = Map(args.map)
    times, odom, scans = load_scans(args.inspection)
    keys = [0]
    for i, q in enumerate(odom):
        last = odom[keys[-1]]
        if np.linalg.norm(q[:2] - last[:2]) >= 0.4 or abs(angle_delta(q[2], last[2])) >= np.deg2rad(20):
            keys.append(i)
            if len(keys) == 5:
                break
    seeds = coarse_search(grid, scans[0])
    static = rank(grid, seeds, [scans[0]], [])
    output = {"first_scan": static, "keyframe_elapsed_s": times[keys].tolist(),
              "odometry_xy_distance_m": np.linalg.norm(odom[keys, :2], axis=1).round(4).tolist()}
    if len(keys) == 5:
        moving = rank(grid, seeds, [scans[i] for i in keys[:3]], [scans[i] for i in keys[3:]])
        output["moving"] = moving
        # Engineering gate: two new viewpoints, score floor, gap to next distinct mode.
        if moving[0]["score"] >= 0.50 and moving[0]["score"] - moving[1]["score"] >= 0.10:
            output["initial"] = moving[0]["pose"]
        tracks = track_modes(grid, seeds, odom, scans, keys)
        output["tracks"] = tracks
        if tracks[0]["score"] >= 0.50 and tracks[0]["score"] - tracks[1]["score"] >= 0.10:
            output["tracking_initial"] = tracks[0]["pose"]
    output["wall_s"] = round(time.perf_counter() - start, 3)
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
