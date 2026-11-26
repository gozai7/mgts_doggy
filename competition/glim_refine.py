"""Smooth horizontal GLIM corrections from repeat observations of vertical planes."""

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from . import bagio, cloud
from .cold_start import moved
from .trajectory import interpolate


def _submaps(bag, poses, mount, knots, holdout_phase=0):
    blocks = [[[] for _ in knots] for _ in range(2)]
    with bagio.reader(bag) as reader:
        channels = [c for c in reader.connections if c.topic == "/competition/lidar"]
        if not channels:
            raise ValueError("Normalized bag has no /competition/lidar topic")
        for index, (channel, _, raw) in enumerate(
            reader.messages(connections=channels)
        ):
            msg = reader.deserialize(raw, channel.msgtype)
            xyz, fields = cloud.unpack(msg)
            times = bagio.ns(msg) * 1e-9 + fields["time"].astype(np.float64)
            valid = np.isfinite(times) & np.isfinite(xyz).all(axis=1)
            xyz, times = xyz[valid], times[valid]
            if not len(times):
                continue
            left = max(0, np.searchsorted(poses[:, 0], times.min()) - 1)
            right = np.searchsorted(poses[:, 0], times.max()) + 1
            if right - left < 2 or left >= len(poses) - 1:
                continue
            track, valid = interpolate(poses[left:right], times, max_gap=0.15)
            if valid.any():
                xyz = (
                    Rotation.from_quat(track[valid, 3:]).apply(moved(xyz[valid], mount))
                    + track[valid, :3]
                )
                k = np.argmin(abs(knots - times.mean()))
                blocks[int(index % 5 == holdout_phase)][k].append(
                    np.column_stack((xyz, times[valid]))
                )
    return [[_planes(group) for group in split] for split in blocks]


def _planes(group):
    if not group:
        return np.empty((0, 3)), np.empty((0, 3)), np.empty(0)
    rows = np.vstack(group)
    _, ids = np.unique(
        np.floor(rows[:, :3] / 0.11).astype(np.int64), axis=0, return_index=True
    )
    rows = rows[ids[:: max(1, int(np.ceil(len(ids) / 16000)))]]
    xyz = rows[:, :3]
    if len(xyz) < 12:
        return np.empty((0, 3)), np.empty((0, 3)), np.empty(0)
    d, idx = cKDTree(xyz).query(xyz, k=12)
    near = xyz[idx]
    near -= near.mean(axis=1)[:, None, :]
    values, axes = np.linalg.eigh(np.einsum("nki,nkj->nij", near, near))
    normals = axes[:, :, 0]
    good = (
        (abs(normals[:, 2]) < 0.25)
        & (values[:, 0] < 0.12 * values[:, 1])
        & (values[:, 1] > 0.12 * values[:, 2])
        & (d[:, -1] < 0.85)
    )
    return xyz[good], normals[good], rows[good, 3]


def _project(block, pivot, correction):
    xyz, normals = block[:2]
    angle = correction[2]
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    moved_xyz, moved_normals = xyz.copy(), normals.copy()
    moved_xyz[:, :2] = (xyz[:, :2] - pivot) @ rot.T + pivot + correction[:2]
    moved_normals[:, :2] = normals[:, :2] @ rot.T
    return moved_xyz, moved_normals


def _pairs(blocks, centers, knots):
    return [
        (i, j)
        for i in range(len(knots))
        for j in range(i)
        if knots[i] - knots[j] > 15
        and np.linalg.norm(centers[i] - centers[j]) < 7
        and min(len(blocks[i][0]), len(blocks[j][0])) >= 30
    ]


def _matches(projected, trees, pairs, centers, corrections):
    for i, j in pairs:
        dst, dn = projected[j]
        src, sn = projected[i]
        # Cap each pair's influence and sample the full source submap.
        ids = np.linspace(0, len(src) - 1, min(900, len(src)), dtype=int)
        d, ix = trees[j].query(src[ids], distance_upper_bound=0.65)
        ids, ix = ids[np.isfinite(d)], ix[np.isfinite(d)]
        keep = abs(np.einsum("ij,ij->i", sn[ids], dn[ix])) > 0.90
        if keep.sum() < 18:
            continue
        a, b, n = src[ids[keep]], dst[ix[keep]], dn[ix[keep]]
        residual = np.einsum("ij,ij->i", a - b, n)

        # Both rotated endpoints use the same target normal in the linearization.
        def jac(points, k, normals):
            v = points[:, :2] - centers[k] - corrections[k, :2]
            return np.column_stack(
                (normals[:, :2], v[:, 0] * normals[:, 1] - v[:, 1] * normals[:, 0])
            )

        yield i, j, residual, jac(a, i, n), -jac(b, j, n)


def _fit(blocks, centers, knots, pairs):
    corrections = np.zeros((len(knots), 3))
    dim = corrections.size
    eye = np.eye(dim).reshape(len(knots), 3, dim)
    # Weak zero prior plus first/second time differences retain GLIM in unobservable axes.
    factor = np.vstack(
        [
            (eye * np.array([1.5, 1.5, 9])[:, None]).reshape(-1, dim),
            (np.diff(eye, axis=0) * np.array([9, 9, 32])[:, None]).reshape(-1, dim),
            (np.diff(eye, n=2, axis=0) * np.array([12, 12, 40])[:, None]).reshape(
                -1, dim
            ),
            1000 * eye[0],
        ]
    )
    regularizer = factor.T @ factor
    for _ in range(10):
        projected = [_project(b, c, u) for b, c, u in zip(blocks, centers, corrections)]
        trees = [cKDTree(b[0]) for b in projected]
        hessian = regularizer.copy()
        rhs = -regularizer @ corrections.ravel()
        for i, j, r, ji, jj in _matches(projected, trees, pairs, centers, corrections):
            w = np.minimum(1.0, 0.075 / np.maximum(abs(r), 1e-12))
            ii, jj_idx = slice(i * 3, i * 3 + 3), slice(j * 3, j * 3 + 3)
            hessian[ii, ii] += (ji.T * w) @ ji
            hessian[jj_idx, jj_idx] += (jj.T * w) @ jj
            off = (ji.T * w) @ jj
            hessian[ii, jj_idx] += off
            hessian[jj_idx, ii] += off.T
            rhs[ii] -= ji.T @ (w * r)
            rhs[jj_idx] -= jj.T @ (w * r)
        step = np.linalg.solve(hessian, rhs).reshape(-1, 3)
        corrections += step
        if np.max(abs(step)) < 0.001:
            break
    return corrections


def _validate(blocks, transformed, pairs):
    """Compare the same held-out cross-visit points before and after registration."""
    trees = [cKDTree(b[0]) for b in blocks]
    new_trees = [cKDTree(b[0]) for b in transformed]
    before, after, align_before, align_after = [], [], [], []
    wall_orientations, pair_changes = [], []
    for i, j in pairs:
        src, sn = blocks[i][:2]
        dst, dn = blocks[j][:2]
        if min(len(src), len(dst)) < 10:
            continue
        selected = np.arange(0, len(src), 4)
        d, ix = trees[j].query(src[selected])
        ok = (d < 0.65) & (abs(np.einsum("ij,ij->i", sn[selected], dn[ix])) > 0.90)
        ids, ix = selected[ok], ix[ok]
        if len(ids) < 15:
            continue
        x, y = transformed[i][0][ids], transformed[j][0]
        dist, jj = new_trees[j].query(x)
        before.extend(d[ok])
        after.extend(dist)
        raw_error = abs(np.einsum("ij,ij->i", src[ids] - dst[ix], dn[ix]))
        new_error = abs(np.einsum("ij,ij->i", x - y[jj], transformed[j][1][jj]))
        align_before.extend(raw_error)
        align_after.extend(new_error)
        wall_orientations.extend(abs(dn[ix, 0]) > 0.85)
        pair_changes.append(np.median(new_error) - np.median(raw_error))

    def stats(v):
        return np.percentile(v, [50, 90, 95]).round(5).tolist() if len(v) else []

    angles = np.asarray(wall_orientations, dtype=bool)
    by_normal = {}
    for label, mask in [("near_x", angles), ("other", ~angles)]:
        by_normal[label] = {
            "count": int(mask.sum()),
            "before_m": stats(np.asarray(align_before)[mask]),
            "after_m": stats(np.asarray(align_after)[mask]),
        }
    return {
        "heldout_points_across_pairs": len(before),
        "heldout_pairs": len(pair_changes),
        "pairs_median_worse": int(np.count_nonzero(np.asarray(pair_changes) > 0)),
        "worst_pair_median_change_m": max(pair_changes, default=None),
        "by_normal": by_normal,
        "quantiles": [50, 90, 95],
        "nn_before_m": stats(before),
        "nn_after_m": stats(after),
        "normal_before_m": stats(align_before),
        "normal_after_m": stats(align_after),
    }


def refine(bag, poses, mount):
    """Return corrected TUM rows and a JSON-compatible quality report."""
    if (
        poses.ndim != 2
        or poses.shape[1] != 8
        or not len(poses)
        or not np.isfinite(poses).all()
        or np.any(np.diff(poses[:, 0]) <= 0)
        or np.any(np.linalg.norm(poses[:, 4:], axis=1) < 1e-9)
        or mount.shape != (4, 4)
        or not np.isfinite(mount).all()
    ):
        raise ValueError("Invalid GLIM TUM poses or lidar mount")
    if poses[-1, 0] - poses[0, 0] <= 15:
        return poses.copy(), {
            "accepted": False,
            "reason": "Recording too short for separated visits",
        }
    knots = np.linspace(
        poses[0, 0], poses[-1, 0], max(2, round((poses[-1, 0] - poses[0, 0]) / 4) + 1)
    )
    centers = interpolate(poses, knots)[0][:, :2]
    train, test = _submaps(bag, poses, mount, knots)
    pairs = _pairs(train, centers, knots)
    if not pairs:
        return poses.copy(), {
            "accepted": False,
            "reason": "No overlapping vertical-plane submaps",
        }
    corr = _fit(train, centers, knots, pairs)
    out = poses.copy()
    ts = poses[:, 0]

    def warp(xyz, times):
        shifts = np.column_stack([np.interp(times, knots, v) for v in corr.T])
        pivot = np.column_stack([np.interp(times, knots, v) for v in centers.T])
        rot = Rotation.from_euler("z", shifts[:, 2])
        delta = np.column_stack((xyz[:, :2] - pivot, np.zeros(len(xyz))))
        projected = xyz.copy()
        projected[:, :2] = rot.apply(delta)[:, :2] + pivot + shifts[:, :2]
        return projected, rot

    out[:, 1:4], rotation = warp(poses[:, 1:4], ts)
    out[:, 4:] = (rotation * Rotation.from_quat(poses[:, 4:])).as_quat()
    projected_test = []
    for points, normals, point_times in test:
        if len(points):
            projected, rot = warp(points, point_times)
            projected_test.append((projected, rot.apply(normals)))
        else:
            projected_test.append((points, normals))
    diagnostics = _validate(test, projected_test, pairs)
    diagnostics.update(
        submaps=len(knots),
        possible_pairs=len(pairs),
        max_shift_m=float(np.linalg.norm(out[:, 1:3] - poses[:, 1:3], axis=1).max()),
        max_yaw_deg=float(np.rad2deg(abs(corr[:, 2])).max()),
        knots_s=(knots - knots[0]).tolist(),
        corrections_xy_yaw=corr.tolist(),
    )
    enough = (
        diagnostics["heldout_points_across_pairs"] >= 100
        and diagnostics["heldout_pairs"] >= 3
    )
    if enough:
        normal_old, normal_new = (
            diagnostics["normal_before_m"],
            diagnostics["normal_after_m"],
        )
        nn_old, nn_new = diagnostics["nn_before_m"], diagnostics["nn_after_m"]
        accepted = (
            normal_new[0] < normal_old[0]
            and normal_new[1] < normal_old[1]
            and nn_new[1] <= nn_old[1]
        )
        reason = (
            "Held-out cross-visit agreement improved"
            if accepted
            else "Held-out agreement did not improve"
        )
    else:
        accepted, reason = False, "Insufficient held-out cross-visit support"
    diagnostics.update(accepted=accepted, reason=reason)
    return (out if accepted else poses.copy()), diagnostics
