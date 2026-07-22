# Kaleidoscope extraction requirements

## Purpose

Move the offline trajectory editor implementation into
`multi_purpose_mpc_ros/tools/kaleidoscope/` so it can later be published and
run by other Automotive AI Challenge users from the organizer Docker image.

## Scope

- Move the trajectory editor GUI and its Python-only support modules.
- Keep the existing `ros2 run multi_purpose_mpc_ros trajectory_editor` and
  `pure_pursuit_trajectory_editor` commands working.
- Keep existing internal import paths working during migration.
- Do not move MPC runtime nodes, V2X tools, maps, or trajectory data.
- Do not introduce a host-specific absolute path.

## Constraints

- The repository checkout and current directory layout may be used to discover
  default trajectory and map files.
- ROS 2 is a launcher/integration concern; the extracted editor core must not
  gain a ROS node dependency.
- Existing trajectory validation and safe-save behavior must be preserved.

## Definition of done

- The canonical editor implementation lives below `tools/kaleidoscope/`.
- The package can be imported as `kaleidoscope`.
- Legacy `multi_purpose_mpc_ros.tools.trajectory_*` imports remain compatible.
- Existing trajectory-related Python tests pass.
- The ROS package installs the extracted Python package.
