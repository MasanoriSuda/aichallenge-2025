# Results

## Verification

- `make autoware-build`: passed, 25 packages.
- `multi_purpose_mpc_ros`: 54/54 test targets passed, 2090 assertions,
  zero errors and zero failures.
- Single-authority source contract: 70/70 passed.
- Bounded two-vehicle run: `output/20260829-033033`.

## Dynamic evidence

The counters below are candidate evaluations, not unique artifacts.

| Domain | feedback attempted | projected | nonlinear continuation | complete proof |
|---|---:|---:|---:|---:|
| d1 | 1282 | 1282 | 915 | 915 |
| d2 | 445 | 445 | 443 | 443 |
| total | 1727 | 1727 | 1358 | 1358 |

All 1358 nonlinear continuations that were constructed also passed the shared
measured-to-control wall, continuation wall, timed dynamic-obstacle and Follow
proof path.  There were zero downstream wall, dynamic-obstacle, Follow or
course-frame rejections in this run.

The remaining 369 evaluations failed while reconstructing an exact nonlinear
continuation.  The diagnostic windows identify actuator-envelope rejection and
initial-lateral-bound rejection.  A projected first steering command is
therefore insufficient when the rest of the prepared input/state sequence was
solved from an older state.

## Root-cause decision

The next boundary is not Mission lifetime, wall clearance, solver tolerance or
a new fallback.  It is the asynchronous feedback connector:

```text
old preparation solve
  -> old state/input sequence
  -> project only the first steering command
  -> later sequence remains inconsistent with the measured state
  -> exact nonlinear continuation rejection
```

This matches the AS-RTI split: preparation may remain asynchronous, but a
feedback phase must rebind the optimization to the latest state and previously
published input before a result can become executable.

## Production impact

None.  A feedback-corrected observation still reports production
`SteeringUnreachable`, cannot construct `Proof`, and cannot be published or
marked executed.

## Next Slice

Implement an observation-only latest-state feedback QP using the existing
seven-state formulation and immutable prepared problem.  It must produce a new
exact trajectory and pass the same physical proof.  Production promotion is a
later atomic change that deletes elapsed-suffix-only candidate adoption.
