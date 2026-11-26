import csv

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from competition import bagio
from competition.config import pose7, transform
from competition.go2_full_route import registered_prefix
from competition.trajectory import save
from tools.go2_export import export


def test_prefix_uses_only_received_samples_and_bounded_first_anchor():
    t = 1_795_605_722_371_275_663
    a_from_odom = transform([4, -3, 1, *Rotation.from_euler("z", 90, degrees=True).as_quat()])
    at_first = transform([2, 0, 0, 0, 0, 0, 1])
    lock = {"stamp_ns": t, "initial_pose": pose7(a_from_odom @ at_first)}
    at_next = transform([3, 0, 1, 0, 0, 0, 1])
    pending = [(t + 70_000_000, at_next), (t + 210_000_000, at_next)]
    actual = registered_prefix(pending, lock, a_from_odom, t, t + 27_000_000)
    assert [n for n, _ in actual] == [t, *(n for n, _ in pending)]
    np.testing.assert_allclose(actual[0][1], lock["initial_pose"])
    np.testing.assert_allclose(actual[1][1], pose7(a_from_odom @ at_next))
    for absent in [None, t + 31_000_000]:
        assert [n for n, _ in registered_prefix(pending, lock, a_from_odom, t, absent)] == [
            n for n, _ in pending
        ]
    recorded = [(t, at_first), *pending]
    assert len(registered_prefix(recorded, lock, a_from_odom, t, t + 27_000_000)) == len(recorded)


def test_bag_reveals_history_at_lock_and_only_appends_online_poses(tmp_path):
    start = 1_795_605_722_371_275_663
    times = [start + dt for dt in (0, 70_000_000, 140_000_000, 210_000_000,
                                    280_000_000, 1_350_000_000, 1_420_000_000)]
    # The second scan had no odometry, including after confirmation.
    valid = [times[0], *times[2:]]
    rows = np.asarray([[n * 1e-9, float(i), 0, 0, 0, 0, 0, 1] for i, n in enumerate(valid)])
    online = rows[2:]
    save(tmp_path / "trajectory.tum", online)
    save(tmp_path / "full_route.tum", rows)
    with (tmp_path / "tracking.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(("stamp_ns", "source", "full_route_source"))
        for n in times:
            source = "uninitialized" if n < times[3] else "lidar"
            full_source = {times[0]: "registered_odometry", times[2]: "registered_lidar"}.get(n, source)
            writer.writerow((n, source, full_source))
    (tmp_path / "map.pcd").write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n"
    )
    export(tmp_path)
    with bagio.reader(tmp_path / "result_bag") as reader:
        paths, live = [], []
        for channel, arrival, data in reader.messages():
            msg = reader.deserialize(data, channel.msgtype)
            if channel.topic == "/competition/result/full_route":
                stamps = [bagio.ns(pose) for pose in msg.poses]
                assert stamps[-1] == bagio.ns(msg) == arrival
                assert all(stamp <= arrival for stamp in stamps)
                paths.append((arrival, stamps))
            if channel.topic == "/competition/result/base_pose":
                live.append(bagio.ns(msg))
    assert live == times[3:]
    assert paths == [(times[3], valid[:3]), (times[5], valid[:5]), (times[6], valid)]
    rows[-1, 1] += 0.1
    save(tmp_path / "full_route.tum", rows)
    with pytest.raises(ValueError, match="unchanged"):
        export(tmp_path)
