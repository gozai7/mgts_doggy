import numpy as np
import yaml

from sim.lidar_suite import PLANS, limits, path_samples, scan, surfaces


def test_gallery_symmetry_and_late_landmark():
    def geometry(name):
        plan = yaml.safe_load((PLANS / f"{name}.yaml").read_text())
        boxes, _ = surfaces(plan)
        return [limits(box) for box in boxes], plan

    symmetric, plan = geometry("symmetric-gallery")
    landmark, other = geometry("landmark-gallery")
    assert plan["routes"] == other["routes"]
    visible = []
    for x, y, yaw in path_samples(plan["routes"]["inspection"])[::100]:
        pose = (x, y, yaw)
        first = scan(pose, symmetric, [], plan["ceiling"])
        mirror = scan((-x, -y, yaw + np.pi), symmetric, [], plan["ceiling"])
        np.testing.assert_allclose(first, mirror, atol=1e-8)
        with_rack = scan(pose, landmark, [], plan["ceiling"])
        visible.append(first.shape != with_rack.shape or not np.allclose(first, with_rack))
    assert not any(visible[:15]) and any(visible[15:])
