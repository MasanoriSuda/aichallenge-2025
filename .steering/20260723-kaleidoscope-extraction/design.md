# Kaleidoscope extraction design

## Layout

```text
multi_purpose_mpc_ros/
├── tools/kaleidoscope/
│   ├── README.md
│   ├── pyproject.toml
│   └── kaleidoscope/
│       ├── __init__.py
│       ├── __main__.py
│       └── trajectory_*.py
└── multi_purpose_mpc_ros/tools/
    └── trajectory_*.py  # temporary import-compatible aliases
```

The implementation uses package-relative imports. `ament_cmake_python`
installs `kaleidoscope` alongside `multi_purpose_mpc_ros`. Compatibility
modules alias the canonical modules rather than copying their names so test
monkeypatching and module globals continue to behave correctly.

## Path discovery

The existing editor already checks ROS package shares and then walks source
parents for `env/` and the sibling `aichallenge_submit_launch/` package. The
new nested location remains discoverable by that parent walk, so no checkout
absolute path is introduced.

## Compatibility

- ROS executable names do not change.
- CSV schemas and editor CLI arguments do not change.
- Existing Python import paths are retained through aliases.
- `v2x_position_editor` continues to consume the compatibility import.
