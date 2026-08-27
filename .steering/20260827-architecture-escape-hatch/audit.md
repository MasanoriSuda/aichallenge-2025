# Audit: Architecture escape-hatch platform

## Observed process failure

The repository can trace violations inside the current architecture, but has
no mandatory boundary that demotes the current Mission/candidate/SQP stack to
one hypothesis after repeated failure-family work. Existing logs also cannot
replay the exact problem because the immutable numerical and physical inputs
are not serialized as one artifact.

## Earliest violated process invariant

An architectural conclusion must be reproducible from the same immutable
input. The current evidence preserves rich diagnostics but not every input
required to compare A--D. Continuing production patches from those diagnostics
would therefore repeat local optimization without a controlled alternative.

## Root producer

The development policy defines one canonical formulation as the target but
does not distinguish the single-authority invariant from replaceable planner,
Mission, convexification and solver hypotheses. The audit workflow has no
mandatory escape-hatch trigger or central rejected-experiment memory.

## Downstream masks

- repeated steering Slice documents can preserve local reasoning while hiding
  the absence of a cross-method comparison;
- extensive runtime diagnostics can look replay-complete although exact QP,
  wall-grid and warm-start payloads are missing;
- `all methods failed` can be overclassified as physical infeasibility when an
  offline local solve merely failed to find a solution.

## Approved repair

Add governance and offline evidence tooling only. Production controller code,
authority and runtime configuration remain frozen in this Slice.

## Dynamic status

Unknown until a complete Pass/Return failure snapshot is captured. Existing
run logs are explicitly registered as incomplete evidence and cannot satisfy
the A--D acceptance gate.

## Verification

- Focused host test: 6/6 passed.
- Snapshot manifest and central registry validation: passed.
- `make autoware-build`: 25 packages built successfully.
- Existing package suite: 47/47 CTest targets, 1,938 tests, zero failures.
- Forced test-enabled reconfigure registered target 48; the new target passed
  6/6 tests in Docker.
- `git diff --check`: passed.

The initial package suite was generated before the new pytest target and did
not count it. A forced test-enabled reconfigure then registered and executed
the target with the source-tree `PYTHONPATH`.

## Production authority audit

This Slice changes no controller source or runtime config. New code is an
offline Python validator/classifier installed as a tool. It has no ROS
publisher, solver, plan store or production-adoption API.
