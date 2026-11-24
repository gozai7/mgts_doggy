"""Geometry and passive lidar controls for localization stress scenes."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import yaml
from PIL import Image
from scene_bags import intersections, save_reference_map, scan_ranges, trajectory
from world import SCENES, layout_for


def scan(scene, x, y, yaw=0):
    return scan_ranges(
        layout_for(scene, "inspection"), [x, y, yaw], np.random.default_rng(12), np.random.default_rng(34)
    )


for scene in SCENES:
    for role in ("mapping", "inspection"):
        poses = np.asarray(list(trajectory(layout_for(scene, role)["route"])))
        np.testing.assert_allclose(poses[0, :2], poses[-1, :2])
        steps = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
        assert steps.max() <= 0.105 and steps.sum() > 75

# Neither the laser nor its noise can uniquely distinguish these two poses.
np.testing.assert_allclose(scan("corridor", -1, -1.5), scan("corridor", 1, -1.5))
np.testing.assert_allclose(scan("colonnade", -2.5, 0), scan("colonnade", 2.5, 0))
glass, opaque = scan("atrium", 0, 0), scan("atrium-opaque", 0, 0)
valid = np.isfinite(glass) & np.isfinite(opaque)
assert np.any(valid & (glass == opaque)) and np.any(glass[valid] - opaque[valid] > 1)
assert np.any(np.isinf(glass) & np.isfinite(opaque))
# A panel behind an opaque wall must not alter that beam.
rays = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]])
d, c = intersections([[2, 0, 1, 1], [4, 0, 1, 1]], [0, 0], rays)
np.testing.assert_array_equal(d, [1.5, np.inf, np.inf])
assert c[0] == 1
layout = {"boxes": [[2, 0, 1, 1], [4, 0, 1, 1]], "glass": [[4, 0, 1, 1]]}
for panels in (layout["glass"], []):
    layout["glass"] = panels
    ranges = scan_ranges(layout, [0, 0, 0], np.random.default_rng(1), np.random.default_rng(1))
    assert abs(ranges[180] - 1.5) < 0.05

with TemporaryDirectory() as tmp:
    folder = Path(tmp)
    save_reference_map(folder, layout_for("atrium", "mapping"))
    grid = np.asarray(Image.open(folder / "map.pgm"))
    config = yaml.safe_load((folder / "map.yaml").read_text())
    ox, oy, _ = config["origin"]
    res = config["resolution"]
    for x, y, value in [(0, 1.5, 0), (0, 0, 254), (-10.3, 0, 205)]:
        assert grid[-1 - int((y - oy) / res), int((x - ox) / res)] == value
