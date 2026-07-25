# Localization Scope

[日本語](README.ja.md) | English

Localization Scope is an offline localization analysis tool for Automotive AI
Challenge runs. It overlays a reference trajectory CSV with GNSS and EKF
positions from rosbag, generates a single-run HTML report, and compares exactly
two Baseline/Candidate runs.

## Supported environment

Use the organizer-provided AI Challenge Docker image and preserve the original
repository layout, as required by Kaleidoscope:

```text
aichallenge/workspace/src/aichallenge_submit/
  multi_purpose_mpc_ros/tools/localization_scope/
```

The tool is not a ROS node. Run it in the Autoware command container because bag
decoding uses `rosbag2_py`, `rclpy`, and `rosidl_runtime_py`. HTML generation
does not require Plotly, pandas, or NumPy.

## Integrating into another participant repository

Placing `tools/localization_scope/` at the documented location is sufficient
for direct source execution:

```bash
cd /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope
python3 -m localization_scope --help
```

To enable `ros2 run multi_purpose_mpc_ros localization_scope`, participants
must also make the following changes outside this tool directory:

1. Install the `localization_scope` Python package from `CMakeLists.txt`.
2. Add `scripts/localization_scope` to `install(PROGRAMS ...)`.
3. Add `rclpy`, `rosbag2_py`, `rosidl_runtime_py`, `sensor_msgs`, and
   `rosbag2_storage_mcap` runtime dependencies to `package.xml` when missing.
4. Copy the executable wrapper to
   `multi_purpose_mpc_ros/scripts/localization_scope`.
5. Optionally copy `test/test_localization_scope.py` and register it with
   `ament_add_pytest_test`.

The exact CMake, package manifest, wrapper, test, and build snippets are
documented in [README.ja.md](README.ja.md). Steering files are development
records and are not required to run the tool.

## Quick start

```bash
cd /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope

python3 -m localization_scope init /path/to/run/run-metadata.json

python3 -m localization_scope report /path/to/run/ \
  --output-dir /path/to/run/localization-report

python3 -m localization_scope compare \
  /path/to/baseline-run/ /path/to/candidate-run/ \
  --output-dir /path/to/comparison

python3 -m localization_scope catalog /path/to/runs/ \
  --output-dir /path/to/catalog
```

The single-run command writes `run-manifest.json`, `summary.json`, and
`report.html`. The comparison command writes `comparison-summary.json` and
`comparison.html`. The catalog command writes a browser selector for single-run
and two-run Baseline/Candidate views.

## Interpretation

The trajectory is a target path, not ground truth. EKF-to-trajectory error mixes
vehicle tracking error and localization error. Record GNSS pose, EKF pose, raw
IMU, vehicle velocity, steering, and the runtime trajectory together to make
the discrepancy easier to isolate.

Missing topics do not abort report generation. Unsupported metrics are shown as
`N/A`, with warnings explaining the limitation.

## `make dev` localization workflow

The primary use case is participant-side localization debugging with `make dev`,
followed by offline rosbag analysis:

```text
change localization/config
  -> run make dev
  -> record rosbag
  -> inspect a single-run report
  -> compare Baseline and Candidate runs
```

Ensure rosbag recording is enabled and includes GNSS fix/pose, raw and corrected
IMU, vehicle velocity, steering status, EKF input/output, runtime trajectory,
and control command topics. In this repository, recording is configured in:

```text
aichallenge/workspace/src/aichallenge_system/
  autostart_orchestrator_py/config/autostart_orchestrator.param.yaml
```

Set `enable_rosbag: true` for the development run and add the recommended topics
listed in [README.ja.md](README.ja.md). Repositories using another recorder
must capture equivalent data and override topic names in `run-metadata.json`.

Recommended tests are stationary, straight constant-speed, left/right constant
turn, acceleration/braking, solo lap, and finally multi-vehicle race runs.
Record an unchanged Baseline first, then change one localization item for the
Candidate. Keep AWSIM version, trajectory, controller configuration, target
speed, test type, lap count, and start condition fixed where possible.

See [README.ja.md](README.ja.md) for the complete topic list, metadata contract,
defaults, and current limitations.

## License

Apache License 2.0
