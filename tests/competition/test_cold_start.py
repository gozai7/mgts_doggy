import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from competition import bagio, cloud
from competition.cold_start import initialize


@pytest.mark.parametrize("duplicate", [False, True])
def test_unknown_start(tmp_path, config, duplicate):
    rng = np.random.default_rng(12)
    # Perpendicular walls and an off-centre pillar distinguish position and heading.
    line = np.linspace(-3, 3, 81)
    xy = np.vstack([np.c_[line, line*0+2], np.c_[line*0+3, line],
                    np.c_[line*.2-.7, line*0-.6], np.c_[line*0-1.3, line*.12-.8]])
    scene = np.vstack([np.c_[xy, np.full(len(xy), z)] for z in (.3, .8, 1.3)])
    angle = Rotation.from_euler("z", 73, degrees=True)
    t = np.array([11., -7., 0.])
    model = angle.apply(scene) + t
    if duplicate:
        model = np.vstack([model, model + [14., 0., 0.]])
    name = tmp_path / "map.pcd"
    header = (f"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(model)}\nHEIGHT 1\nPOINTS {len(model)}\nDATA binary\n")
    name.write_bytes(header.encode() + model.astype("<f4").tobytes())
    bag = tmp_path / "B"
    config["extrinsics"]["T_base_lidar"] = [0, 0, 0, 0, 0, 0, 1]
    times = [1_000_000_000 + 300_000_000*i for i in range(9)]
    bagio.write_messages(bag, (("/competition/lidar", n,
                        cloud.pack(scene + rng.normal(0, .003, scene.shape), n, "lidar")) for n in times))
    if duplicate:
        with pytest.raises(ValueError, match="No unambiguous"):
            initialize(name, bag, config)
    else:
        result = initialize(name, bag, config)
        pose = result["initial_pose"]
        assert result["stamp_ns"] == times[0]
        assert np.linalg.norm(pose[:3] - t) < .12
        assert (angle.inv() * Rotation.from_quat(pose[3:])).magnitude() < .04
