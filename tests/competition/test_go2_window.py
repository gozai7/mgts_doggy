import numpy as np
from scipy.spatial.transform import Rotation

from competition.config import pose7
from competition.go2_tracking import MapTracker
from competition.go2_window import WindowTracker


def test_window_corrects_height_and_tilt_then_propagates_and_expires():
    # Three measured orthogonal planes constrain the 6D fit.
    x, y = np.meshgrid(np.linspace(-2, 2, 39), np.linspace(-2, 2, 39))
    xy = np.column_stack((x.ravel(), y.ravel()))
    plane = np.column_stack((xy, np.zeros(len(xy))))
    walls = np.column_stack((xy, np.full(len(xy), 2.0)))
    points = np.vstack((plane, walls[:, [2, 0, 1]], walls[:, [1, 2, 0]]))
    initial = np.eye(4)
    initial[:3, 3] = [0.025, 0.04, 0.15]
    initial[:3, :3] = Rotation.from_euler("xy", [2.0, 3.0], degrees=True).as_matrix()
    target = MapTracker(points, initial.copy())
    win = WindowTracker(target)
    body = np.eye(4)
    scan = win.prepare(body, points)
    first = 10**9
    pending = [
        (first + dt, body, scan) for dt in (75_000_000, 145_000_000, 220_000_000)
    ]
    rows, lidar = win.replay_prefix(
        pending,
        {"stamp_ns": first, "initial_pose": pose7(initial)},
        first,
        first + 27_000_000,
    )
    assert [t for t, _ in rows] == [first, *(t for t, _, _ in pending)]
    assert lidar == {pending[1][0]}
    assert abs(rows[-1][1][2]) < abs(initial[2, 3])
    assert (
        Rotation.from_quat(rows[-1][1][3:]).magnitude()
        < Rotation.from_matrix(initial[:3, :3]).magnitude()
    )
    later_body = body.copy()
    later_body[0, 3] = 0.4
    pose, source = win.update(first + 3 * 10**9, later_body, np.empty((0, 3)))
    assert source == "odometry" and not win.window
    np.testing.assert_allclose(pose, target.map_from_odom @ later_body)
