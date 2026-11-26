# Independent localization backends

The default commands and original configuration still run the GLIM-ready baseline. The experimental
profiles share the same preprocessing, immutable map bundle, trajectory formats and replay code.
Mapping selection is `mapping.backend: glim|rko_lio` (or `make mapping BACKEND=...`).
`BACKEND` remains the default for old configurations. Use a different mapping output for each backend.

| Profile | Map correction | Relative motion |
|---|---|---|
| `baseline` (L0) | lidar_localization_ros2 | Original configured predictor |
| `beluga` (L1) | Beluga SE(2), 3D VDB likelihood field | Existing recorded base odometry |
| `ndt_kiss` (L2) | lidar_localization_ros2 | PRBonn KISS-ICP |
| `beluga_kiss` (L3) | Beluga SE(2), 3D VDB likelihood field | PRBonn KISS-ICP |

`existing` must be an actual Odometry stream for Beluga. A constant-velocity scan-matching predictor
does not provide that stream. The implementation refuses to silently substitute KISS or GT.
The existing stream can be kinematic or a previously available estimator; describe its provenance.

## Build and run

The full Dockerfile includes both mapping backends by default, Beluga and KISS. The GPU target keeps
GLIM's CUDA build; the new localization components run on CPU. To build only the original mapper,
set `BUILD_MAPPING_BACKENDS=glim`. Source SHAs and licenses are in `THIRD_PARTY.lock.json`.

```bash
make build
# Or: BUILD_MAPPING_BACKENDS=glim make build-gpu
make mapping BACKEND=glim CONFIG=configs/competition/my_mapping.yaml
make occupancy CONFIG=configs/competition/my_mapping.yaml

# Fill a copy of configs/competition/localization.yaml with measured sensor contracts.
# localization.map_bundle points to the output root containing mapping/ and occupancy/.
make localization PROFILE=baseline CONFIG=configs/competition/my_localization.yaml
make localization PROFILE=beluga CONFIG=configs/competition/my_localization.yaml
make localization PROFILE=ndt_kiss CONFIG=configs/competition/my_localization.yaml
make localization PROFILE=beluga_kiss CONFIG=configs/competition/my_localization.yaml
make benchmark-localization CONFIG=configs/competition/my_localization.yaml
```

Once maps exist, a smaller image can build/run all localizers without building either SLAM:

```bash
docker compose -f compose.yaml -f compose.localizers.yaml build competition
make localization PROFILE=ndt_kiss CONFIG=configs/competition/my_localization.yaml \
  COMPOSE='docker compose -f compose.yaml -f compose.localizers.yaml'
```

If packages.ros.org is unavailable, `ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`
selects an alternative at build time. No downloads occur during localization; Compose processing
uses `network_mode: none`. APT versions are not a snapshot, so retain the tested Docker image.
Inputs are mounted read-only. Failed runs and changed checkpoints are never overwritten; set
`localization.run_name` to a new directory name to rerun a profile with different settings.

`make odometry` runs KISS on B (unlike the historical `make baseline`, which runs it on A).
KISS is an independent first pass. It processes B irrespective of global localization success,
and its timestamped relative trajectory is cached/reused for L2 and L3. Offline two-pass processing
avoids processing-order races between a scan's odometry and its map registration.

## Frames, timestamps and motion

Experimental odometry profiles use `map -> odom -> base -> lidar/imu`.
Replay owns `odom -> base`, and its supervisor owns `map -> odom`; the NDT localizer's TF
broadcast is disabled while its internal map/odom anchor is retained. Beluga publishes hypotheses
on `/beluga/pose` and `/beluga/particles`, not authoritative TF. Run profile comparisons separately,
or use separate ROS_DOMAIN_IDs: two replay processes must not own the same TF chain.

KISS's LiDAR-frame pose is converted to the physical base using the measured mount. NDT consumes
its pose through the upstream `use_odom_tf_prediction` interface. KISS does not fill twist, so
remapping its messages into the twist-based `use_odom` predictor would not supply motion.
Original `kinematic`, `imu` and `constant_velocity` predictors keep their original behavior.

Replay advances `/clock` and waits for a result for each sensor timestamp. Integer `stamp_ns`
sidecars preserve timestamps even at large Unix epochs. Existing odometry is interpolated at
scan timestamps only inside its support and across bounded gaps (`odometry.max_gap_s`). Missing
odometry is reported, never extrapolated. QoS is configured explicitly for scan and diagnostics.

Point times are normalized once by the existing adapter. No artificial point times are generated.
Select `point_time.mode: none` for unknown time conventions or already-deskewed input; KISS and
NDT deskew then stay disabled. A `PointCloud2` in a world/odom frame is not a sensor-frame scan.
New KISS profiles also default `odometry.deskew` to false. Enabling it requires verified
`point_time.header_reference: end` and point offsets ending near zero: upstream KISS deskews to
the sweep end but preserves the message header. Positive start-relative offsets must not silently
be labelled as end-reference odometry. Beluga observations with verified per-point offsets are
warped once to the header pose using bounded spatial-odometry interpolation, including the sensor
lever arm. Points outside motion support are omitted and counted; no motion is extrapolated.

## What Beluga estimates

The checked upstream is `e6323da8c7bee863e29950910737bb3502af1306` (package version 2.2.0).
The ROS `ndt_amcl_node_3d` is not a mature six-degree-of-freedom global localizer: its differential
motion model flattens SE(3) to SE(2), its NDT wrapper cannot sample global free-space states,
and that wrapper enforces fixed particle count. The library's VDB model explicitly supports
SE(2) particles with 3D observations, which this adapter uses.

The filter, omnidirectional motion model (including sideways motion), likelihood field,
resampling, KLD particle adaptation and recovery probability estimator are unmodified Beluga.
The adapter adds ROS IO, a free-space sampler, deterministic seeding and diagnostics.
It does not implement another particle filter or registration algorithm.

`map.pcd` becomes an OpenVDB level set using upstream's topology-to-level-set approach. World
coordinates, scale and origin stay unchanged. OccupancyGrid free cells, eroded by robot clearance
and a jitter margin, constrain initial XY. Yaw is uniform over a full turn. Unknown cells are
excluded. Conversion is keyed by content hashes, voxel resolution, occupancy, clearance and
Beluga revision; cached artifact hashes are checked before reuse. VDB conversion does not require
another SLAM. NDT's alternative HDF5 representation is not used by this backend.

Particles estimate **x, y, yaw**. A gravity-aligned spatial odometry pose is decomposed into
its planar pose and residual z/roll/pitch. That residual and `T_base_lidar` transform the 3D
observations; the same residual lifts the planar result back to the physical base. Height is
anchored using the explicitly measured `beluga.initial_base_height_m` in map coordinates.
These vertical/tilt components remain conditional on odometry, not independently observed
global 6DoF estimates. They are never silently set to zero.
The published 6x6 covariance combines the planar filter block with conditional odometry
z/roll/pitch variances; unavailable cross-correlations are not estimated. It is not a joint 6DoF
posterior or a claim that the common-LiDAR errors are independent.

For KISS, configure measured initial base roll/pitch in `odometry.initial_roll_pitch_rad`;
KISS's arbitrary initial axes do not supply gravity. Confirm `odometry.gravity_aligned: true`
only after checking this reference. KISS itself does not consume IMU. A gravity reference obtained
from another estimator must be disclosed when assessing independence. The same LiDAR supplies
KISS and Beluga observations, so their errors are correlated; vary `beluga.motion_noise` when
evaluating sensitivity rather than assuming statistical independence.

Unknown-start runs never call the existing BBS/ICP initializer for Beluga. An explicitly supplied
`initial_pose` selects a separate approximate-start experiment and is recorded as such.

## Confidence, recovery and artifacts

Beluga reports particle count, covariance, update time, geometric overlap, reinitializations and
sampled particle distributions. Covariance concentration plus overlap only produce a candidate:
the common supervisor also checks relative-motion consistency and consecutive confirmations.
An isolated jump is rejected. A new consistent sequence can replace the old map/odom anchor.
KISS is not reset on relocalization. A confirmed anchor can continue a bounded odometry-only
trajectory; without any confirmed anchor, relative motion never becomes a global trajectory.

Beluga can reset globally through `/beluga/global_localization`; replay requests reset after a
lost track or an unsuccessful approximate start with a cooldown. NDT retains upstream local
recovery and reinitialization diagnostics; automatic orchestration of its map-wide G2 service is
not implemented here. Particle mode extraction and automatic switching between backends are also
not implemented. A concentrated filter can still be confidently wrong. Effective sample size
before resampling is not exposed by this adapter; no fabricated ESS is exported.

Results are under `output/profiles/<profile-or-run_name>/`:

- `trajectory.tum`, `trajectory.csv`, `odometry.tum`, `localization_status.csv`;
- `metrics.json`, `resolved_config.yaml`, `frames.json`, `resource_usage.json`, `logs/`;
- `particle_diagnostics.csv`, `particles.jsonl` for Beluga;
- `registration_diagnostics.csv` for NDT/GICP and `relocalization_events.csv`;
- original upstream estimates in `upstream_trajectory.tum`, alignment messages in JSONL.

CSV contains one row per received input scan, including missing or unconfirmed estimates.
Sources distinguish `map_based`, `odometry_only`, `unconfirmed` and `missing`; final TUM contains
available confirmed/fallback poses only. A LiDAR outage has no invented scans or interpolated
global poses, and its elapsed time is counted as missing. KISS cannot supply motion during a
complete LiDAR outage. Resource figures sample the process group every 100 ms; short-lived
children may be missed, GPU memory remains null. Localizer wall time and map preparation time
are recorded separately; a reused odometry cache is identified.

RViz includes Beluga hypotheses/covariance and relative odometry. `python tools/compare_localization.py
/results/...` writes a standalone `trajectories.svg`, with dashed odometry-only segments and
preserved gaps. It refuses to overlay different map hashes without an explicit external alignment.

## Experiments and decision gates

`benchmark_plan.json` is saved before algorithms run. It fixes input identities, map hash,
seeds, scenarios and quantitative gates. Default engineering gates require at least a 10 percentage
point initialization success gain or 15% P95 improvement, at most 2 percentage points coverage
loss, and at most 600 seconds localizer wall time. Accuracy/false-initialization gates require
independent GT; coverage or concentration alone do not establish them. Small seed counts provide
preliminary engineering evidence, not a statistically established advantage.

The example matrix includes unknown start, 50/25% density, scan loss, 0.5/2/5-second gaps and IMU
faults. Fault injection operates on copies with saved settings and seed. Add explicit
`initial_pose` scenarios for wrong/approximate start, and separate `bag` entries for changed objects,
repeated geometry and corridors when such recordings exist. These scene semantics cannot be
manufactured by a launch option. IMU faults only test a profile that consumes IMU; they cannot
change pre-recorded odometry. Test normal and unknown-start modes separately.

Provide `benchmark.ground_truth` and one `benchmark.T_reference_map` obtained from A or calibration
to compute APE/RPE; it is applied identically to every profile, without fitting B. Otherwise
absolute accuracy stays null. `comparison.json` retains failed trials and their errors.

## Supplied hackathon bags

Source: https://github.com/ivr0ma/hackathon, downloaded Git LFS objects:

- mapping ZIP: `e2f0f28321eae5f75224c3ee09e27fad80588489911e0e53ec2383a88e940c86`, 114170436 bytes;
- localization ZIP: `9ba414629b33947e18ef145b9c16f42e7c8d24b25209345d7253b0c2f5a7727d`, 101996018 bytes.

Mapping: 121.076 s, 1688 raw LiDAR scans. Localization: 109.867 s, 1526 raw scans.
Both contain `/utlidar/cloud` in `utlidar_lidar`, `/utlidar/cloud_base` in `base_link`,
`/utlidar/cloud_deskewed` in `odom`, `/utlidar/robot_odom`, IMU and LowState. They contain no TF
or explicit map topic. Recorder timestamps lag sensor timestamps by about 80 seconds;
the normalizer uses corrected sensor stamps, rather than treating that as an IMU offset.

The LiDAR mount in `hackathon.yaml` was recovered from exact paired ring/time/intensity identities
in raw/base clouds and verified on ten scans from each bag (RMSE around 5–8e-8 m):
`[0.282160018, 0, -0.024670004, 0, 0.9914583559, 0, 0.1304236502]`.
Reproduce with `tools/infer_cloud_mount.py --raw-topic /utlidar/cloud --base-topic /utlidar/cloud_base`.
This does not calibrate IMU. Its full extrinsics remain null; GLIM/RKO mapping with this profile
is deliberately blocked pending measured calibration. Initial gravity tilt for KISS/Beluga is
from the recorded base odometry, not an independent gravity sensor. Body height uses A's median
base z. No B global XY/yaw pose is supplied to initialization.

An auxiliary real-data map can be imported from A's already world-framed deskewed stream:

```bash
docker compose -f compose.yaml -f compose.localizers.yaml run --rm competition \
  python tools/import_bag_map.py --config configs/competition/hackathon.yaml \
  --world-cloud /utlidar/cloud_deskewed --output /results/hackathon/provided-map --drop-zero-padding
make localization PROFILE=ndt_kiss CONFIG=configs/competition/hackathon.yaml \
  COMPOSE='docker compose -f compose.yaml -f compose.localizers.yaml'
```

The importer preserves A's coordinates (aliases A `odom` as the experiment `map`), discards explicit
zero padding, voxelizes and exports free space using raw scans and supplied base odometry. It
labels this `provided_bag_world_cloud`, with **no offline graph optimization**. This is a useful
real-data localization check, not a GLIM/RKO mapping comparison. `/utlidar/robot_odom` is a vendor
LiDAR/IMU estimate; it is not GT and must not be called independent leg kinematics. The conservative
profile disables deskew because timestamp anchoring has not been independently established.

See `LOCALIZATION_TEST_REPORT.md` for what was actually built/run in this environment.
