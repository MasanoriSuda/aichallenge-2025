# Follow retained current-world validation

## Static result

- The focused current-world retained suite passes all 6 cases.
- After rebuilding every package test target, CTest passes all 38 tests.
- `make autoware-build` passes all 25 packages.
- After the final helper-name cleanup, the `mpc_controller_cpp` target recompiles and links.
- `git diff --check` passes.

## Covered evidence

The pure tests prove that a retained Follow plan is accepted only when the current target identity,
observation generation, deterministic target-tube fingerprint, complete target forecast, physical
hard gap and wall path all agree. Target substitution, tube mutation, malformed target progress,
current hard-gap violation and future-stage hard-gap violation fail closed.

Runtime code stores a fresh Follow plan only after the complete canonical command chain succeeds.
When a later eligible cycle has no fresh canonical command, the prior store is evaluated through the
typed current-world proof, canonical retained-candidate builder, normal-authority selector and exact
actuation/prediction reconstruction. The result remains `authority=shadow, selected=0`.

## Commands

```bash
make autoware-build

docker compose run -T --rm --no-deps autoware-build bash -lc \
  'cmake --build /aichallenge/build/multi_purpose_mpc_ros -j2 --target <all-test-targets> && \
   ctest --test-dir /aichallenge/build/multi_purpose_mpc_ros --output-on-failure'

docker compose run -T --rm --no-deps autoware-build bash -lc \
  'cmake --build /aichallenge/build/multi_purpose_mpc_ros --target mpc_controller_cpp -j2'
```

## Dynamic gate

Pending. A useful run must contain all of the following in order:

1. a complete fresh Follow canonical plan is stored;
2. a later eligible Follow cycle has a typed fresh canonical miss;
3. the same target is still current and its complete forecast covers the retained remainder;
4. `Follow retained MPCC shadow` reports `command > 0` and `selected=0`.

If the run contains no such fresh-miss event, it is evidence of neither acceptance nor rejection and
will be recorded as inconclusive. Follow production authority is not promoted by this Slice.
