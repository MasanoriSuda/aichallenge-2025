# Results

## Implemented

- Added one pure candidate resolver shared by branch solve and atomic handoff.
- Candidate order is complete selected Mission, receding prefix, then selected
  progressive prefix.
- Kept `progressive_entry=true` and the bounded prefix metadata intact.
- Re-runs the existing progressive-entry admission in the live callback before
  a worker-selected prefix may start a new entry.
- Active same/cross-side replacements still pass through the existing runtime
  prefix admission and no-return gates.
- Extended-MPCC status now reports candidate source, prefix flag and solve or
  rejection reason for each side.

## Static verification

### Build

```text
docker compose run -T --rm --no-deps autoware-build
Summary: 25 packages finished
[build_autoware] Build successful.
```

The only stderr output was the existing setuptools `setup.py install`
deprecation warning.

### Tests

```text
colcon test --packages-select multi_purpose_mpc_ros
100% tests passed, 0 tests failed out of 28
```

The new resolver tests cover:

- complete Mission precedence;
- receding-prefix fallback;
- selected-progressive-prefix fallback;
- invalid side and wrong-side rejection.

`git diff --check` also passed.

## Expected dynamic evidence

The previous run always reported `dual=L0/R0`. A successful bridge should now
show an attempted branch such as:

```text
dual=L1/1/receding-prefix/p1/.../ok,...,select=1/...
```

`L1` or `R1` proves that a bounded prefix reached the production extended QP.
`p1` confirms it remained a prefix. A failed solve now includes its actual
failure reason instead of being conflated with a missing complete Mission.

Dynamic performance and contact behavior remain unverified in this change;
the next `make dev2` run is the acceptance check.
