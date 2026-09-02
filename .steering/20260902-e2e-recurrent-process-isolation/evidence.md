# Evidence

## Static and unit acceptance

- Host controller suite: `89 passed`.
- Host E2E launch contract: `3 passed`.
- Docker contract suite: `47 passed`.
- Docker executor suite: `4 passed`.
- Docker E2E launch contract: `3 passed`.
- `make autoware-build`: 25 packages completed successfully.
- Python compilation and `git diff --check`: passed.

The subprocess contract test proves that the worker reports an exact artifact
SHA, exact self-described runtime configuration, `OPENBLAS_NUM_THREADS=1`, a
distinct process ID and the same loaded parameter count as the parent-side
contract verifier.  Wrong artifact identity fails closed during worker
initialization.

## Scope boundary

This evidence does not promote recurrent steering authority.  The production
publisher, production checkpoint, spatial authority, longitudinal settings and
all dynamic Gate thresholds remain unchanged.  Single- and three-vehicle
dynamic acceptance are still required.

## Test harness note

The first combined `colcon test` invocation reported a transient Python module
collection error for the controller package while the same installed module
was directly importable.  Running the installed package tests directly inside
the same Docker image passed (47 + 4 tests), and the launch package passed in
both modes.  This is recorded as a test-runner/overlay observation rather than
hidden as a product failure.
