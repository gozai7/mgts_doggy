import numpy as np
from scipy.spatial.transform import Rotation

from tools.go2_global_seed import height_weights


def test_holdout_weights_discard_floor_and_ignore_odometry_origin_and_yaw():
    xy = np.linspace(-2, 2, 28)
    x, y = np.meshgrid(xy, xy)
    floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -0.32)))
    identity = [0, 0, 0, 0, 0, 0, 1]
    assert height_weights(floor, identity) is None
    assert height_weights(floor[:5], identity) is None
    tangent = np.array([0.82, 0.57])
    a, h = np.meshgrid(xy, np.linspace(0.08, 1.35, 12))
    wall = np.column_stack((a.ravel()[:, None] * tangent, h.ravel()))
    points = np.vstack((floor, wall))
    weights = height_weights(points, identity)
    alternative_frame = [342, -17, 6, *Rotation.from_euler("z", 1.21).as_quat()]
    assert np.all(weights[: len(floor)] == 0)
    np.testing.assert_allclose(weights.sum(), 1.0)
    np.testing.assert_allclose(weights, height_weights(points, alternative_frame), atol=1e-12)


def test_layered_proposals_ignore_initial_odometry_origin_and_yaw():
    from tools.go2_global_seed import candidates

    rng = np.random.default_rng(3)
    source = rng.uniform([-2.0, -0.8, -0.2], [1.0, 1.5, 1.2], size=(140, 3))
    tilt = Rotation.from_euler("yx", [8.0, -4.0], degrees=True)
    model = tilt.apply(source) + [3.0, 5.0, 0.35]
    ref = np.array([[0.0, 3.0, 5.0, 0.35]])
    transformed = Rotation.from_euler("z", 117.0, degrees=True) * tilt
    a = candidates(model, source, ref, first_odom=[0, 0, 0, *tilt.as_quat()])
    b = candidates(model, source, ref, first_odom=[35, -9, -17, *transformed.as_quat()])
    for (score, p), (other_score, q) in zip(a[:10], b[:10]):
        np.testing.assert_allclose(score, other_score)
        np.testing.assert_allclose(p, q, atol=1e-12)
