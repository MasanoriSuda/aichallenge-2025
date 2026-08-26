# Requirements: current-cycle authority continuity experiment

## Status

Rejected by dynamic evidence. This steering slice records a falsified
hypothesis; it makes no production-code change.

## Question

Are same-intent retained-authority losses merely asynchronous proof holes that
can be repaired by running the existing exact six-state admission transaction
inside the current callback?

## Constraints

- Preserve the single six-state MPCC normal authority.
- Preserve exact problem, solution, physical proof, current-world proof, and
  serialized-command identity joins.
- Do not add a legacy controller, grace, lease, timeout, fallback, or tuning.
- Do not weaken wall or dynamic-obstacle evidence.
- The 25 ms callback budget is an acceptance requirement.

## Evidence inputs

- Baseline: `output/20260826-170617`.
- Experimental run: `output/20260826-180846`.

## Acceptance / rejection rule

The idea is acceptable only if it bridges a transient missing proof without
retrying a solver on genuine current-world rejection and without creating
callback overruns. It must be rejected if physical blockers remain or solver
failure makes the callback exceed 25 ms.

## Result

The experiment joined 49 transactions across D1/D2, but rejected 644. During
solver failure it repeatedly consumed about 60--70 ms in the callback. The
production and test changes were removed.
