# Results

## Frozen failure classification

The first failing ShiftOut episode in `output/20260830-133656` loses normal
authority at decision 1336 because the rebuilt terminal Stop suffix is
wall-infeasible while its immediate normal publisher interval remains clear.
At that snapshot, persistent, stateless, rough-candidate and diagonal
architecture arms all fail; changing side at decision 1336 is therefore not a
supported repair.

The retained proof did own an exact Stop state trajectory one cycle earlier,
but the production adapter reduced it to a boolean.  Emergency Stop then
generated another base-track command.  This is a certificate/publisher owner
split, not evidence that the old successor is safe to execute.

## Implemented observation boundary

- Terminal Stop now records every nonlinear integration input together with
  its exact end state.
- Retained proof rejects a state trajectory detached from its actuation shape.
- Production authority transports the immutable evidence as observation-only
  data.
- Evidence is promoted only after the canonical command matches the serialized
  command and crosses the existing publication boundary.
- The next control origin is sampled by the pure
  `mpcc_certified_stop_successor_observation` module.
- Production command selection and Emergency Stop behavior are unchanged.

## Static verification

- `make autoware-build`: passed (Docker build, 25 packages).
- `test_mpcc_certified_stop_successor_observation`: 4/4 passed.
- `test_mpcc_rate_resolved_physical_adapter`: 20/20 passed.
- `test_mpcc_rate_resolved_retained_revalidation`: 51/51 passed.
- `test_mpcc_rate_resolved_command_candidate`: 14/14 passed.
- `test_single_authority_source_contract.py`: 85/85 passed with third-party
  pytest plugin autoload disabled.

The first host `ctest` invocation was not a code failure: ROS's
`ament_cmake_test` Python module was not on that container command's Python
path.  Directly executing the built gtest binaries in the same project image
passed.

## Dynamic exit criterion

Run `make dev2` and inspect `Certified Stop successor join`:

- `result=sampled` with small state deltas supports a later Slice which makes
  the certified Stop successor executable;
- identity/time failure indicates scheduling or lifecycle provenance;
- large pose/speed/steering deltas indicate model/publisher mismatch and block
  any production connection.

