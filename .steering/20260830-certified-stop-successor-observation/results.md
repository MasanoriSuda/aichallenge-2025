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

## Dynamic run: `output/20260830-142647`

The observation joined the next control origin 1,282 times. ShiftOut and Pass
were both represented, so the join is not limited to Cruise. Identity and time
provenance generally survived the publication boundary.

Position, yaw and speed errors were usually small enough to support further
classification. Steering error, however, repeated in discrete values close to
one or more control-rate steps (for example 0.010713, 0.014284, 0.017855,
0.021514 and 0.025172 rad). This is structured rather than random noise.

The run therefore does **not** authorize production use of the successor. It
shows that the causal join exists while leaving two competing explanations:

1. terminal Stop integration and serialized steering are offset by a sample;
2. the observation compares the exact successor against the wrong one of the
   controller's physical, response, command-origin and committed steering
   states.

The run also exposed an observability defect: one log per control cycle created
more than a thousand lines. The follow-up Slice aggregates telemetry and
separates steering owners before any authority change.
