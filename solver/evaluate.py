"""Compare both passes in the frame fixed by the first mapping observation."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from geometry import compose, inverse
from PIL import Image, ImageDraw


def trajectory(folder, name="trajectory.csv"):
    rows = np.loadtxt(folder / name, delimiter=",", skiprows=1, ndmin=2)
    if len(rows) == 0 or rows.shape[1] != 4 or not np.isfinite(rows).all():
        raise ValueError(f"invalid {folder / name}")
    if np.any(np.diff(rows[:, 0]) < 0):
        raise ValueError(f"out-of-order timestamps in {folder / name}")
    return rows


def interpolate(rows, at):
    return np.column_stack(
        [np.interp(at, rows[:, 0], v) for v in (rows[:, 1], rows[:, 2], np.unwrap(rows[:, 3]))]
    )


def transform(rows, frame):
    return np.column_stack((rows[:, 0], [compose(frame, pose) for pose in rows[:, 1:]]))


def recording_window(truth, window):
    start, end = window
    if truth[-1, 0] < start or end < truth[0, 0]:
        raise ValueError("ground truth and lidar recording do not overlap")
    times = truth[:, 0]
    times = np.unique(
        np.r_[max(start, times[0]), times[(times >= start) & (times <= end)], min(end, times[-1])]
    )
    return np.column_stack((times, interpolate(truth, times)))


def score(estimate, truth, max_gap):
    t = truth[:, 0]
    e = estimate[:, 0]
    j = np.searchsorted(e, t).clip(0, len(e) - 1)
    valid = (t >= e[0]) & (t <= e[-1]) & ((t == e[j]) | (e[j] - e[(j - 1).clip(0)] <= max_gap))
    dt = np.diff(t)
    covered = valid[:-1] & valid[1:]
    boundaries = np.flatnonzero(np.diff(np.r_[False, ~covered, False]))
    gaps = [t[end] - t[start] for start, end in boundaries.reshape(-1, 2)]
    report = dict(
        coverage=float(dt[covered].sum() / dt.sum()) if dt.sum() else float(valid[0]),
        longest_gap_s=float(max(gaps, default=0)),
    )
    if valid.any():
        delta = interpolate(estimate, t[valid]) - truth[valid, 1:]
        d = np.linalg.norm(delta[:, :2], axis=1)
        heading = np.rad2deg(np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2])))
        report.update(
            rmse_m=float(np.sqrt(np.mean(d**2))),
            p95_m=float(np.percentile(d, 95)),
            yaw_rmse_deg=float(np.sqrt(np.mean(heading**2))),
        )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mapping", type=Path)
    parser.add_argument("inspection", type=Path)
    parser.add_argument("--truth-a", type=Path)
    parser.add_argument("--truth-b", type=Path)
    parser.add_argument("--max-gap", type=float, default=1.0)
    args = parser.parse_args()
    if bool(args.truth_a) != bool(args.truth_b):
        parser.error("supply both truth sessions or neither")
    a, b = args.mapping, args.inspection
    meta_a, meta_b = [json.loads((f / "run.json").read_text()) for f in (a, b)]
    if args.max_gap <= 0:
        parser.error("--max-gap must be positive")
    if meta_a["mode"] != "map" or meta_b["mode"] != "localize":
        parser.error("expected completed mapping and localization runs")
    if Path(meta_b["map"]).resolve() != (a / "map.yaml").resolve():
        parser.error("inspection used another map")
    am, loc, od = trajectory(a), trajectory(b), trajectory(b, "odometry.csv")
    config = yaml.safe_load((a / "map.yaml").read_text())
    image = Image.open(a / config["image"]).convert("L")
    grid = np.asarray(image)
    overlay = image.convert("RGB")
    draw = ImageDraw.Draw(overlay)
    res, (ox, oy, _) = config["resolution"], config["origin"]

    def pixels(x, y):
        return (x - ox) / res, image.height - 1 - (y - oy) / res

    def path(rows, color):
        x, y = pixels(rows[:, 1], rows[:, 2])
        if len(rows) > 1:
            draw.line(list(zip(x, y)), fill=color, width=2)

    hits = np.load(b / "scan_hits.npy")
    hits = hits[(hits[:, 0] >= loc[0, 0]) & (hits[:, 0] <= loc[-1, 0])]
    px, py, yaw = interpolate(loc, hits[:, 0]).T
    x, y = hits[:, 1:].T
    u, v = pixels(px + np.cos(yaw) * x - np.sin(yaw) * y, py + np.sin(yaw) * x + np.cos(yaw) * y)
    draw.point(list(zip(u, v)), fill="#00a4be")
    path(am, "#986bb1")
    if meta_b["initial"] is not None:
        baseline = transform(od, compose(meta_b["initial"], inverse(od[0, 1:])))
        path(baseline, "#d87f00")
    else:
        baseline = None
    path(loc, "#2762bb")
    report = dict(mapping=meta_a["counts"], inspection=meta_b["counts"], absolute_errors=None)
    report["inspection"]["max_pose_gap_s"] = float(np.diff(loc[:, 0]).max(initial=0))
    report["inspection"]["max_pose_step_m"] = float(
        np.linalg.norm(np.diff(loc[:, 1:3], axis=0), axis=1).max(initial=0)
    )
    if args.truth_a:
        ta, tb = trajectory(args.truth_a), trajectory(args.truth_b)
        anchor = am[0]
        if not ta[0, 0] <= anchor[0] <= ta[-1, 0]:
            raise ValueError("truth A does not cover map anchor timestamp")
        world_to_map = compose(anchor[1:], inverse(interpolate(ta, [anchor[0]])[0]))
        ref_a = recording_window(
            transform(ta, world_to_map), meta_a.get("scan_window", [ta[0, 0], ta[-1, 0]])
        )
        ref_b = recording_window(
            transform(tb, world_to_map), meta_b.get("scan_window", [tb[0, 0], tb[-1, 0]])
        )
        path(ref_b, "#249d39")
        report["world_to_map"] = world_to_map.tolist()
        report["absolute_errors"] = dict(
            mapping=score(am, ref_a, args.max_gap),
            inspection=score(loc, ref_b, args.max_gap),
            odometry=score(baseline, ref_b, args.max_gap) if baseline is not None else None,
        )
        raw_a = trajectory(a, "odometry.csv")
        ends = am[[0, -1], 0]
        if np.all((ends >= ta[0, 0]) & (ends <= ta[-1, 0])):
            gt_start, gt_end = interpolate(ref_a, ends)
            od_start, od_end = interpolate(raw_a, ends)
            predicted_end = compose(compose(gt_start, inverse(od_start)), od_end)
            report["mapping_endpoints"] = dict(
                true_separation_m=float(np.linalg.norm(gt_end[:2] - gt_start[:2])),
                odometry_end_error_m=float(np.linalg.norm(gt_end[:2] - predicted_end[:2])),
                optimized_end_error_m=float(np.linalg.norm(gt_end[:2] - am[-1, 1:3])),
            )
        layout = json.loads((args.truth_a / "layout.json").read_text())
        iy, ix = np.indices(grid.shape)
        mx, my = ox + (ix + 0.5) * res, oy + (image.height - iy - 0.5) * res
        dx, dy, angle = inverse(world_to_map)
        wx = dx + math.cos(angle) * mx - math.sin(angle) * my
        wy = dy + math.sin(angle) * mx + math.cos(angle) * my
        expected = np.zeros_like(grid, dtype=bool)
        for x, y, w, h in layout["boxes"]:
            expected |= (abs(wx - x) <= w / 2) & (abs(wy - y) <= h / 2)
        x0, y0, x1, y1 = layout["bounds"]
        known = (grid != 205) & (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
        occupied = grid == 0
        union = (known & (occupied | expected)).sum()
        report["map_quality"] = dict(
            observed_fraction=min(1.0, float(known.sum() * res**2 / ((x1 - x0) * (y1 - y0)))),
            occupied_iou=float((known & occupied & expected).sum() / union) if union else None,
        )
    overlay.resize((overlay.width * 2, overlay.height * 2), Image.Resampling.NEAREST).save(b / "overlay.png")
    output = json.dumps(report, indent=2, allow_nan=False)
    (b / "report.json").write_text(output)
    print(output)


if __name__ == "__main__":
    main()
