"""Regression: recover a track even when every AMCL anchor is in the wrong place."""
import numpy as np
from nav_msgs.msg import OccupancyGrid

from refine import refine


def scene(duplicate):
    resolution = 0.05
    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 240, 120, resolution
    cells = np.zeros((120, 240), dtype=np.int8)
    # Irregular landmarks, all expressed at cell centres. The second scene duplicates the layout.
    rng = np.random.default_rng(512)
    row, col = rng.integers(10, 110, size=(2, 65))
    cells[row, col] = 100
    if duplicate:
        cells[row, col + 120] = 100
    grid.data = cells.ravel().tolist()
    landmarks = (np.column_stack((col, row)) + 0.5) * resolution
    t = np.arange(41) * 0.1
    yaw = 0.1 + 0.13*t
    truth = np.column_stack((t, 2.2 + 0.25*t, 2.1 + 0.06*t*t, yaw))
    scans = []
    for _, x, y, heading in truth:
        c, s = np.cos(heading), np.sin(heading)
        scans.append((landmarks - [x, y]) @ [[c, -s], [s, c]])
    odom = truth.copy()
    odom[:, 1] += 0.03*t
    amcl = odom.copy()
    amcl[:, 1] += 6
    return grid, odom, amcl, scans, truth


grid, odom, prior, scans, truth = scene(False)
result, modes = refine(grid, odom, prior, scans, None)
assert modes != ["amcl"] and np.array_equal(result[:, 0], truth[:, 0])
assert np.max(np.linalg.norm(result[:, 1:3] - truth[:, 1:3], axis=1)) < 0.12, modes

grid, odom, prior, scans, truth = scene(True)
result, modes = refine(grid, odom, prior, scans, None)
assert modes == ["ambiguous"], modes
np.testing.assert_array_equal(result, prior)
