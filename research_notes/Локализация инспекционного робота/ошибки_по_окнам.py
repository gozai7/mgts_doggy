"""Read frozen runs and score the local office-B benchmark inside the project's solver image."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/mgts/solver")
from evaluate import compose, interpolate, inverse, recording_window, trajectory, transform


def main():
    first_a = trajectory(Path("/runs/final-A"))[0]
    ref_at_anchor = interpolate(trajectory(Path("/truth/office-A")), [first_a[0]])[0]
    world_to_map = compose(first_a[1:], inverse(ref_at_anchor))
    ref = recording_window(transform(trajectory(Path("/truth/office-B")), world_to_map), [30.5, 77.7])
    elapsed = ref[:, 0] - ref[0, 0]
    for route in sys.argv[1:]:
        est = trajectory(Path("/runs") / route)
        if est[0, 0] > ref[0, 0] or est[-1, 0] < ref[-1, 0] or np.max(np.diff(est[:, 0])) > 1:
            raise ValueError("This office-B window comparison requires complete coverage.")
        delta = interpolate(est, ref[:, 0]) - ref[:, 1:]
        position = np.linalg.norm(delta[:, :2], axis=1)
        heading = abs(np.rad2deg(np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))))
        print(route, "from_s to_s RMSE_m median_m P95_m yaw_RMSE_deg")
        for lo, hi in [(0, 18), (18, 30.5), (30.5, np.inf)]:
            mask = (elapsed >= lo) & (elapsed < hi)
            values = (np.sqrt(np.mean(position[mask] ** 2)), *np.percentile(position[mask], [50, 95]),
                      np.sqrt(np.mean(heading[mask] ** 2)))
            print(lo, min(hi, elapsed[-1]), *(round(float(x), 4) for x in values))
        outside = (position > 0.25) | (heading > 10)
        bad_count = np.r_[0, np.cumsum(outside)]
        end = np.searchsorted(elapsed, elapsed + 5)
        good = (end < len(elapsed)) & ((bad_count[end] - bad_count[:-1]) == 0)
        idx = np.flatnonzero(good)
        print("first_stable_5s", float(elapsed[idx[0]]) if idx.size else None)


if __name__ == "__main__":
    main()
