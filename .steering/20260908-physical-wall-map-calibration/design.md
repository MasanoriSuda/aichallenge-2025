# Calibration boundaries

The Unity extraction is read-only and pinned by asset hashes. Rasterization
uses the existing native center-origin convention and exact triangle/cell
SAT. It is an additive update, not larger clearance or learned margin.

Map metadata preserve_occupied_cells bypasses only the old reference/corridor
occupied-island noise filter. The final physical world already uses the raw
normalized image at create_map_ref_path_car_mpc; its occupied cells were not
removed by that filter. The metadata makes candidate geometry agree with it. Threshold normalization stays unchanged. The runtime consumer
and the editor's pure loader read the same metadata. Boolean metadata is
validated rather than treating arbitrary nonempty strings as true.

The isolated actual Map class can be compiled from its source declaration
in a diagnostic harness using OpenCV/yaml-cpp, without ROS startup. The
regression observes native loaded cells, not a duplicated filtering formula.
Production tests retain independent known contact coordinates and editor
preservation checks. A fresh run seals changed map/YAML/controller hashes.
