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
mkdir /runs/imported
cp /runs/A/map.yaml /runs/A/map.pgm /runs/imported/
python3 solver/solve.py localize /data/fixture-B /runs/imported-B --map /runs/imported/map.yaml --profile /mgts/configs/go2.yaml --lidar /inspection/laser --rate 2
python3 solver/evaluate.py /runs/imported /runs/imported-B --truth-root /truth
if python3 solver/evaluate.py /runs/A /runs/imported-B; then exit 1; fi
python3 - <<'PY'
from pathlib import Path
import yaml

path = Path("/runs/imported/map.yaml")
config = yaml.safe_load(path.read_text())
config["origin"][2] = 1.570796327
path.write_text(yaml.safe_dump(config))
PY
if python3 solver/solve.py localize /data/fixture-B /runs/rotated --map /runs/imported/map.yaml 2>/tmp/rotated.err; then exit 1; fi
grep -q 'requires zero origin yaw' /tmp/rotated.err
test ! -e /runs/rotated
if python3 solver/evaluate.py /runs/imported /runs/imported-B 2>/tmp/rotated.err; then exit 1; fi
grep -q 'requires zero origin yaw' /tmp/rotated.err
PYTHONPATH=solver python3 - <<'PY'
import json
from pathlib import Path
import numpy as np
from evaluate import score
from geometry import compose, inverse

report = json.load(open('/runs/B/report.json'))
imported = json.load(open('/runs/imported-B/report.json'))
assert imported['mapping'] is None and imported['absolute_errors'] is None
assert imported['inspection']['forwarded'] == 80
log = Path('/runs/imported-B/ros.log').read_text()
assert log.index('[amcl]: Received a ') < log.index('[amcl]: Initializing with uniform distribution')
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
