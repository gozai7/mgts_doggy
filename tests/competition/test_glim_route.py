import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from competition.config import pose7, transform
from competition.glim_route import align


def pose(x, y, angle):
    return transform([x, y, 0.5, *Rotation.from_euler("z", angle, degrees=True).as_quat()])


def test_one_se3_for_entire_independent_slam_route():
    a_from_odom = pose(5, -3, 32)
    a_from_glim = pose(-2, 4, -63)
    odo = np.array([[1 + i * 0.05, *pose7(pose(2 * i, -i, 10 * i))] for i in range(3)])
    glim = np.array(
        [
            [r[0], *pose7(np.linalg.inv(a_from_glim) @ a_from_odom @ transform(r[1:]))]
            for r in odo[1:]
        ]
    )
    # Preserve SLAM's own displacement even when it disagrees with odometry.
    glim[-1, 1] += 0.75
    lock = dict(
        initial_pose=pose7(a_from_odom @ transform(odo[0, 1:])),
        first_odometry_pose=odo[0, 1:].tolist(),
        stamp_ns=10**9,
        prefix_s=25,
        separated=True,
    )
    route, anchor = align(glim, odo, [dict(separated=False), lock])
    assert anchor["anchor_stamp_s"] == glim[0, 0]
    np.testing.assert_allclose(transform(anchor["T_A_glim_B_xyzw"]), a_from_glim, atol=1e-10)
    for actual, local in zip(route, glim):
        assert actual[0] == local[0]
        np.testing.assert_allclose(
            transform(actual[1:]), a_from_glim @ transform(local[1:]), atol=1e-10
        )
    with pytest.raises(ValueError, match="confirmed"):
        align(glim, odo, [dict(separated=False)])
