"""Register available pre-lock scan odometry once the global start is confirmed."""

from .config import pose7


def registered_prefix(pending, lock, map_from_odom, first_scan_ns, first_odom_ns):
    rows = [(stamp, pose7(map_from_odom @ body)) for stamp, body in pending]
    # prefix() anchors the first scan to the first odometry when the scan leads it.
    if (lock["stamp_ns"] == first_scan_ns and first_odom_ns is not None
            and 0 <= first_odom_ns - first_scan_ns <= 30_000_000
            and (not rows or rows[0][0] != first_scan_ns)):
        rows.insert(0, (first_scan_ns, lock["initial_pose"]))
    return rows


def path_due(stamp, previous):
    return previous is None or stamp - previous >= 1_000_000_000
