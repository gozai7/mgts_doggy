"""Offline SE(2) trajectory refinement on a fixed occupancy grid."""

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from geometry import compose, inverse


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def relative(poses):
    """Body-frame odometry increments between successive rows."""
    xy = np.diff(poses[:, :2], axis=0)
    c, s = np.cos(poses[:-1, 2]), np.sin(poses[:-1, 2])
    return np.column_stack((c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1],
                            wrap(np.diff(poses[:, 2]))))


class DistanceField:
    def __init__(self, grid):
        cells = np.array(grid.data).reshape(grid.info.height, grid.info.width)
        self.free = cells == 0
        self.resolution = grid.info.resolution
        origin = grid.info.origin.position
        self.origin = np.array([origin.x, origin.y])
        self.limit = 0.35  # metres; cap the influence of unmodelled obstacles
        self.field = np.full(cells.shape, self.limit)
        if np.any(cells == 100):
            known = cells >= 0
            distance = distance_transform_edt(cells != 100) * self.resolution
            self.field[known] = np.minimum(self.limit, distance[known])

    def residual(self, pose, points):
        # pose may be one pose, one per point, or a batch of pose hypotheses.
        x, y = points.T
        c, s = np.cos(pose[..., 2]), np.sin(pose[..., 2])
        col = (pose[..., 0] + c*x - s*y - self.origin[0]) / self.resolution - 0.5
        row = (pose[..., 1] + s*x + c*y - self.origin[1]) / self.resolution - 0.5
        return map_coordinates(self.field, [row, col], order=1, mode="constant", cval=self.limit)

    def global_seeds(self, points):
        # Bounded coarse search over free space and yaw. Refine distinct modes on the full track.
        stride = max(1, round(0.20 / self.resolution), int(np.ceil(np.sqrt(self.free.size / 20000))))
        row, col = np.nonzero(self.free[::stride, ::stride])
        if not len(row):
            return []
        xy = self.origin + (np.column_stack((col, row)) * stride + 0.5) * self.resolution
        beams = points[::max(1, int(np.ceil(len(points) / 40)))]
        peaks, costs = [], []
        for angle in np.deg2rad(np.arange(-180, 180, 8)):
            poses = np.column_stack((xy, np.full(len(xy), angle)))
            for chunk in np.array_split(poses, max(1, int(np.ceil(len(poses) / 1000)))):
                errors = self.residual(chunk[:, None], beams)
                costs.append(np.mean(errors**2, axis=1))
            peaks.append(poses)
        peaks, costs = np.vstack(peaks), np.concatenate(costs)
        seeds = []
        for _ in range(4):
            i = np.argmin(costs)
            if not np.isfinite(costs[i]):
                break
            seed = peaks[i]
            seeds.append(seed)
            nearby = ((np.linalg.norm(peaks[:, :2] - seed[:2], axis=1) < 0.75)
                      & (abs(wrap(peaks[:, 2] - seed[2])) < np.deg2rad(25)))
            costs[nearby] = np.inf
        return seeds


# Local correlative search around the odometry prediction. Ties favour smaller corrections.
_OFFSETS = np.stack(np.meshgrid(np.arange(-2, 3) * 0.05, np.arange(-2, 3) * 0.05,
                                np.deg2rad(np.arange(-2, 3) * 2)), axis=-1).reshape(-1, 3)


def track(field, odom, scans, seed, anchor):
    poses = np.empty_like(odom)
    for step in (-1, 1):
        correction = compose(seed, inverse(odom[anchor]))
        for i in range(anchor, -1 if step < 0 else len(poses), step):
            prediction = compose(correction, odom[i])
            hypotheses = prediction + _OFFSETS
            errors = field.residual(hypotheses[:, None], scans[i])
            penalty = 0.02 * np.sum(_OFFSETS**2, axis=1)
            poses[i] = hypotheses[np.argmin(np.mean(errors**2, axis=1) + penalty)]
            correction = compose(poses[i], inverse(odom[i]))
    return poses


def optimize(field, poses, odom, scans):
    # Each laser residual touches one SE(2) pose, each odometry residual touches two.
    count = len(poses)
    owner = np.repeat(np.arange(count), [len(p) for p in scans])
    points = np.vstack(scans)
    pattern = lil_matrix((len(points) + 3*(count-1), 3*count), dtype=bool)
    rows = np.arange(len(points))
    for axis in range(3):
        pattern[rows, 3*owner + axis] = True
    for i in range(count - 1):
        start = len(points) + 3*i
        pattern[start:start+3, 3*i:3*i+6] = True
    travel = relative(odom)
    xy_sigma = 0.035 + 0.15 * np.linalg.norm(travel[:, :2], axis=1)
    motion_sigma = np.column_stack((xy_sigma, xy_sigma, 0.02 + 0.15 * abs(travel[:, 2])))
    # Beams within a scan are correlated. Give dense and sparse scans comparable influence.
    laser_sigma = 0.08 * np.sqrt(np.maximum(1, np.bincount(owner)[owner] / 10))

    def residual(flat):
        p = flat.reshape(count, 3)
        motion = relative(p) - travel
        motion[:, 2] = wrap(motion[:, 2])
        return np.r_[field.residual(p[owner], points) / laser_sigma, (motion / motion_sigma).ravel()]

    result = least_squares(residual, poses.ravel(), jac_sparsity=pattern.tocsr(),
                           loss="soft_l1", max_nfev=30, ftol=1e-4)
    return result.x.reshape(count, 3)


def refine_segment(field, rows, baseline, scans, initial):
    # Include endpoints and sample moving/turning robots more densely than stationary ones.
    keys = [0]
    for i in range(1, len(rows)):
        last = keys[-1]
        delta = rows[i] - rows[last]
        if (delta[0] >= 0.5 or np.linalg.norm(delta[1:3]) >= 0.15
                or abs(wrap(delta[3])) >= 0.10 or i == len(rows)-1):
            keys.append(i)
    keys = np.asarray(keys)
    if len(keys) < 3 or any(len(scans[i]) < 6 for i in keys):
        return baseline, "insufficient_scans"
    odom = rows[keys, 1:]
    train, check = [], []
    for i in keys:
        rays = scans[i]
        held = np.arange(len(rays)) % 3 == 1
        train.append(rays[~held])
        check.append(rays[held])

    def cost(path):
        return np.mean([np.mean(field.residual(p, hits)**2) for p, hits in zip(path, check)])

    def consistent_start(path):
        return initial is None or (np.linalg.norm(path[0, :2] - initial[:2]) <= 0.5
                                   and abs(wrap(path[0, 2] - initial[2])) <= 0.35)

    raw = baseline[keys]
    candidates = [raw]
    anchors = [(i, raw[i]) for i in (0, len(keys)//2, len(keys)-1)]
    if initial is None:
        # The midpoint by distance is less sensitive to a long initial standstill.
        travel = np.r_[0, np.cumsum(np.linalg.norm(np.diff(odom[:, :2], axis=0), axis=1))]
        mid = np.searchsorted(travel, travel[-1] / 2)
        anchors.extend((mid, seed) for seed in field.global_seeds(train[mid]))
    for anchor, seed in anchors:
        candidate = track(field, odom, train, seed, anchor)
        if consistent_start(candidate):
            candidates.append(candidate)
    def distinct(a, b):
        separated = ((np.linalg.norm(a[:, :2] - b[:, :2], axis=1) > 0.75)
                     | (abs(wrap(a[:, 2] - b[:, 2])) > np.deg2rad(25)))
        return np.mean(separated) > 0.5

    # Refine competing modes to the same precision before judging ambiguity:
    # coarse grid quantization alone can break a true symmetry.
    scored = sorted(((cost(path), i, path) for i, path in enumerate(candidates)),
                    key=lambda item: item[0])
    modes = []
    for value, index, path in scored:
        if any(not distinct(path, previous) for _, _, previous in modes):
            continue
        fitted = optimize(field, path, odom, train)
        fit_cost = cost(fitted)
        mode = "amcl" if index == 0 else "scan_match"
        if consistent_start(fitted) and fit_cost < value:
            path, value, mode = fitted, fit_cost, "optimized"
        modes.append((value, mode, path))
        if len(modes) == 3:
            break
    modes.sort(key=lambda item: item[0])
    value, mode, best = modes[0]
    for other_value, _, other in modes[1:]:
        if distinct(best, other) and np.sqrt(other_value) < np.sqrt(value) + 0.01:
            return baseline, "ambiguous"
    if best is raw:
        return baseline, "amcl"

    # Interpolate changes in map <- odom, leaving intermediate measured motion intact.
    corrections = np.array([compose(p, inverse(o)) for p, o in zip(best, odom)])
    corrections[:, 2] = np.unwrap(corrections[:, 2])
    at_scan = np.column_stack([np.interp(rows[:, 0], rows[keys, 0], col) for col in corrections.T])
    result = np.array([compose(c, o) for c, o in zip(at_scan, rows[:, 1:])])
    result[:, 2] = wrap(result[:, 2])
    return result, mode


def refine(grid, odom_rows, amcl_rows, scans, initial):
    odom = np.asarray(odom_rows)
    result = np.array(amcl_rows, dtype=float)
    start = np.searchsorted(odom[:, 0], result[0, 0])
    rows, scans = odom[start:], scans[start:]
    if not np.array_equal(rows[:, 0], result[:, 0]):
        raise ValueError("refinement requires matching scan and trajectory timestamps")
    gaps = ((np.diff(rows[:, 0]) > 1.0) | (np.linalg.norm(np.diff(rows[:, 1:3], axis=0), axis=1) > 1.0)
            | (abs(wrap(np.diff(rows[:, 3]))) > 1.0))
    bounds = np.r_[0, np.flatnonzero(gaps) + 1, len(rows)]
    if initial is not None and start:
        initial = compose(initial, compose(inverse(odom[0, 1:]), rows[0, 1:]))
    field = DistanceField(grid)
    modes = []
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        result[lo:hi, 1:], mode = refine_segment(
            field, rows[lo:hi], result[lo:hi, 1:], scans[lo:hi], initial if lo == 0 else None)
        modes.append(mode)
    return result, modes
