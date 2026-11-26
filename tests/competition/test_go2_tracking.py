import numpy as np
from competition import cloud
from competition.go2_tracking import BodyOdometry


def test_point_offsets_keep_milliseconds_at_unix_epoch():
    """A point taken 64 ms into a translating sweep moves 64 mm on deskew."""
    stamp = 1_795_605_722_371_275_663
    secs = stamp * 1e-9
    odom = BodyOdometry()
    for i in range(-1, 13):
        dt = i * 0.007
        odom.history.append([secs + dt, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    offsets = np.linspace(0, 0.064, 64, dtype=np.float32)
    msg = cloud.pack(np.tile([1.0, 0.0, 0.0], (len(offsets), 1)), stamp, "base_link")
    # cloud.pack's intensity is an adjacent float32 field like raw Go2 point time.
    msg.fields[-1].name = "time"
    msg.data.view("<f4").reshape(-1, 4)[:, 3] = offsets
    _, points = odom.at_scan(msg)
    np.testing.assert_allclose(points[:, 0], 1.0 + offsets, atol=1e-6)
