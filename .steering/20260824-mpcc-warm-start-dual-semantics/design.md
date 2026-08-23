# Design: MPCC warm-start dual semantics

## Root cause

`shift_mpc_warm_start` treats all rows in the dynamics and curvature-rate
blocks as homogeneous stages. They are not.

```text
dynamics row 0      : -x[0] = -measured_state
dynamics row 1..N   : -x[k+1] + A[k]x[k] + B[k]u[k] = c[k]

rate row 0          : curvature[0] in bounds around actual previous curvature
rate row 1..N-1     : curvature[k] - curvature[k-1] in delta bounds
```

The existing generic shift maps old dynamics row 1 to new row 0 and old rate
row 1 to new row 0. Those multipliers belong to different equations. With the
strict physical-row solver contract, every failed cycle in the bounded run was
warm-started and every immediate cold cycle recovered, which is the expected
signature of this mismatch.

## Experimented correction

Keep the current generic stage shift, then clear:

- the first `state_dimension` dynamics dual values;
- the first curvature-rate dual value.

Rows after the boundary already map correctly: new transition/rate stage 1
corresponds to old stage 2 after a one-stage horizon shift. State/input box and
declared trailing blocks keep their existing mapping because their row meaning
does not change at stage zero.

No primal value, QP matrix, bound, solver setting or command authority changed.

## Result

The correction was rejected. It passed all static verification but the bounded
run still produced eight warm-start-only execution-primal rejects. The source
and test edits were therefore removed rather than preserved as another
unproven patch.

The more upstream lifecycle defect is that
`solve_extended_progress_problem()` stores the OSQP-accepted primal/dual as the
next warm start before the caller performs semantic execution-primal
normalization and physical certification. A solution rejected by that later
authority can consequently seed subsequent solves. The next Slice must make
accepted canonical certification, not raw solver success, the publication
boundary for warm-start history.

## Validation sequence

1. Unit test the full primal/dual vector exactly.
2. Run all package tests and build.
3. Run bounded `dev2` with the existing production policy.
4. If warm/cold correlation remains, reject this correction and audit the
   warm-start publication lifecycle before reconsidering Track/Cruise strict
   row normalization.
