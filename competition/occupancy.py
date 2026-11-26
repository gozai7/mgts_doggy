"""Conservative raycast map: unknown cells stay unknown; endpoints are not free."""
from pathlib import Path
from collections import Counter
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from . import bagio, cloud
from .trajectory import read_tum, interpolate, change_body
from .config import transform, write_json


def ray_cells(start, end):
    """Integer Bresenham, excluding the occupied endpoint."""
    x, y = map(int, start); ex, ey = map(int, end)
    dx, dy = abs(ex - x), -abs(ey - y)
    sx, sy = (1 if x < ex else -1), (1 if y < ey else -1)
    err = dx + dy
    while x != ex or y != ey:
        yield x, y
        e = 2 * err
        if e >= dy:
            err += dy; x += sx
        if e <= dx:
            err += dx; y += sy


def grid_from_rays(scans, resolution, min_hits=2, max_cells=16000000):
    hits, misses = Counter(), Counter()
    scans_used = 0
    for origin, endpoints in scans:
        if len(endpoints) == 0:
            continue
        scans_used += 1
        a = np.floor(np.asarray(origin)[:2] / resolution).astype(int)
        cells = np.unique(np.floor(endpoints[:, :2] / resolution).astype(int), axis=0)
        occupied = set(map(tuple, cells))
        free = set()
        for b in cells:
            # Refuse runaway coordinates caused by a bad pose/unit scale.
            if np.max(abs(b - a)) > 10000:
                raise ValueError("Ray exceeds 10000 grid cells; inspect units/trajectory")
            free.update(ray_cells(a, b))
        free -= occupied
        hits.update(occupied); misses.update(free)
    if not hits:
        raise ValueError("No usable obstacle endpoints for occupancy map")
    coords = np.array(list(hits.keys()) + list(misses.keys()))
    low, high = coords.min(axis=0) - 1, coords.max(axis=0) + 1
    shape = high - low + 1
    if int(shape[0]) * int(shape[1]) > max_cells:
        raise ValueError("Occupancy map exceeds configured maximum cells")
    grid = np.full((shape[1], shape[0]), -1, dtype=np.int8)
    for cell, n in misses.items():
        if cell not in hits and n >= 1:
            x, y = np.array(cell) - low; grid[y, x] = 0
    for cell, n in hits.items():
        if n >= min_hits:
            x, y = np.array(cell) - low; grid[y, x] = 100
    return grid, low * resolution, {"scans_used": scans_used, "occupied": int((grid == 100).sum()),
                                   "free": int((grid == 0).sum()), "unknown": int((grid == -1).sum())}


def export_grid(folder, grid, origin, resolution, stamp_ns, map_frame):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid == 0] = 254; image[grid == 100] = 0
    h, w = grid.shape
    (folder / "map.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + image[::-1].tobytes())
    meta = {"image": "map.pgm", "resolution": resolution, "origin": [float(origin[0]), float(origin[1]), 0.0],
            "negate": 0, "occupied_thresh": .65, "free_thresh": .196, "mode": "trinary"}
    (folder / "map.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    T = bagio.STORE.types
    pose = T["geometry_msgs/msg/Pose"](T["geometry_msgs/msg/Point"](float(origin[0]), float(origin[1]), 0.),
                                      T["geometry_msgs/msg/Quaternion"](0., 0., 0., 1.))
    info = T["nav_msgs/msg/MapMetaData"](bagio.time_msg(stamp_ns), float(resolution), w, h, pose)
    msg = T["nav_msgs/msg/OccupancyGrid"](bagio.header(stamp_ns, map_frame), info, grid.ravel())
    bagio.write_messages(folder / "occupancy_bag", [("/map", stamp_ns, msg)])


def build(bag_path, trajectory_path, output, c, pose_frame="base", lidar_topic="/competition/lidar", deskewed=False):
    a = read_tum(trajectory_path)
    if pose_frame == "base":
        a = change_body(a, transform(c["extrinsics"]["T_base_lidar"]))
    elif pose_frame != "lidar":
        raise ValueError("Occupancy pose_frame must be base or lidar")
    cfg = c["occupancy"]
    counts = {"input_scans": 0, "missing_pose": 0, "points_without_pose": 0}

    def scans():
        with bagio.reader(bag_path) as r:
            for conn, _, raw in r.messages(connections=[x for x in r.connections if x.topic == lidar_topic]):
                i = counts["input_scans"]; counts["input_scans"] += 1
                if i % cfg["every_n_scans"]:
                    continue
                msg = r.deserialize(raw, conn.msgtype)
                xyz, arr = cloud.unpack(msg)
                t = bagio.ns(msg) * 1e-9
                p, ok = interpolate(a, [t], cfg["max_pose_gap_s"])
                if not ok[0]:
                    counts["missing_pose"] += 1
                    continue
                origin = p[0, :3]
                # Compensate raw points with the FINAL optimized lidar trajectory.
                # This interpolation is for raster export; it is not an LIO estimator.
                if "time" in arr.dtype.names and not deskewed:
                    poses, valid = interpolate(a, t + arr["time"], cfg["max_pose_gap_s"])
                    counts["points_without_pose"] += int((~valid).sum())
                    xyz = Rotation.from_quat(poses[valid, 3:]).apply(xyz[valid]) + poses[valid, :3]
                else:
                    xyz = Rotation.from_quat(p[0, 3:]).apply(xyz) + origin
                z = xyz[:, 2] - cfg["floor_z_m"]
                xyz = xyz[(z >= cfg["height_m"][0]) & (z <= cfg["height_m"][1])]
                # Nearest endpoint in each angular sector avoids rays through nearer obstacles.
                if len(xyz):
                    delta = xyz[:, :2] - origin[:2]
                    bins = np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + np.pi) / (2 * np.pi) * cfg["angular_bins"]).astype(int)
                    order = np.argsort(np.linalg.norm(delta, axis=1))
                    _, first = np.unique(bins[order], return_index=True)
                    xyz = xyz[order[first]]
                yield origin, xyz
    grid, origin, report = grid_from_rays(scans(), cfg["resolution_m"], cfg["min_hit_scans"], cfg["max_cells"])
    export_grid(output, grid, origin, cfg["resolution_m"], round(a[-1, 0] * 1e9), c["frames"]["map"])
    report.update(counts)
    report["method"] = "raycasting from optimized lidar poses; height-selected nearest endpoints"
    report["raw_point_deskew"] = not deskewed and c["point_time"]["mode"] != "none"
    write_json(Path(output) / "report.json", report)
    return report
