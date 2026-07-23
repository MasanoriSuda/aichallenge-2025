# Kaleidoscope

[日本語](README.ja.md) | English

Kaleidoscope is the Python/Tk offline trajectory editor used by
`multi_purpose_mpc_ros`. Its implementation is kept in this directory so the
tool can later be published independently without moving MPC runtime code.

At this stage, the supported environment is the organizer-provided AI
Challenge Docker image and this repository checkout. The editor is not a ROS
node; ROS 2 currently provides the installed command and package-share lookup.

## Required repository layout

Use Kaleidoscope with the original AI Challenge repository layout unchanged.
Clone or copy this repository into
`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/kaleidoscope/`.
Renaming or moving `multi_purpose_mpc_ros`, its `env/` directory, the sibling
`aichallenge_submit_launch` package, or their map and trajectory files is not
supported. The user is responsible for preserving this layout.

## Run from the existing AI Challenge workspace

Build the workspace once and enter the Autoware command container:

```bash
make autoware-build
make autoware-bash
```

Then run the existing compatible command:

```bash
ros2 run multi_purpose_mpc_ros trajectory_editor
```

To run the extracted package directly from the source checkout inside the
container:

```bash
cd /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/kaleidoscope
python3 -m kaleidoscope \
  --trajectory ../../env/final_ver3/traj_mincurv.csv \
  --osm ../../../aichallenge_submit_launch/map/lanelet2_map.osm \
  --circular
```

The default path discovery still supports the repository layout, so
`python3 -m kaleidoscope` also opens the repository's built-in MPC preset when
the sibling packages and data are present.

## Runtime dependencies

- Python 3.10 or newer
- Tkinter (`python3-tk` on Ubuntu)
- defusedxml
- PyYAML

The editor GUI does not require `rclpy`, ROS topics, or ROS services. Before
publishing example maps or trajectories, document and verify their separate
redistribution terms.

## License

Kaleidoscope is licensed under the [Apache License 2.0](LICENSE).
