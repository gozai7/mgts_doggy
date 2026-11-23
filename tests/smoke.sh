#!/bin/bash
# Run inside the solver image with the tests directory mounted at /tests.
set -eu
mkdir /data /truth /runs
python3 /tests/replay_fixture.py
mkdir /data/capture-A
mv /data/fixture-A /data/capture-A/sensors
printf '%s\n' '{"complete":true,"lidar":"/scan"}' > /data/capture-A/manifest.json
python3 solver/solve.py map /data/capture-A /runs/A --rate 2
python3 solver/solve.py localize /data/fixture-B /runs/B --map /runs/A/map.yaml --profile /data/fixture-B/profile.yaml --initial 1 0 0 --rate 2
python3 solver/evaluate.py /runs/A /runs/B
python3 -c "import json; assert json.load(open('/runs/B/report.json'))['absolute_errors'] is None"
python3 solver/evaluate.py /runs/A /runs/B --truth-a /truth/fixture-A --truth-b /truth/fixture-B
PYTHONPATH=solver python3 - <<'PY'
import json
import numpy as np
from evaluate import score
from geometry import compose, inverse

report = json.load(open('/runs/B/report.json'))
assert report['mapping']['forwarded'] == 80
assert report['absolute_errors']['inspection']['coverage'] > 0.8
assert report['absolute_errors']['mapping']['rmse_m'] < 0.25
assert report['absolute_errors']['inspection']['rmse_m'] < 0.5
assert report['map_quality']['occupied_iou'] > 0.2
assert report['mapping']['imu_samples'] == 80
np.testing.assert_allclose(compose([1, 2, 0.3], inverse([1, 2, 0.3])), 0., atol=1e-12)
truth = np.column_stack((np.arange(6), np.zeros((6, 3))))
partial = score(truth[[0, 1, 4, 5]], truth, 1)
assert partial['coverage'] == 0.4 and partial['longest_gap_s'] == 3
PY
