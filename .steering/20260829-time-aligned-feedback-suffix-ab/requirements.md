# Requirements: time-aligned feedback suffix A/B

## Objective

Determine whether an asynchronous seven-state MPCC preparation can be joined
to the current vehicle state by preserving the candidate's absolute future
stage times and discarding the already consumed prefix.

This is an architecture experiment.  It must not change production authority.

## Frozen observations

- Updating only `x0` in an old final QP mixes control origins and makes narrow
  future progress boxes infeasible.
- A full current-world rebuild solves the deterministic counterexample.
- The upper reference log runs the main GMPCC at roughly 7 Hz and keeps async
  work for tactical alternatives; it does not demonstrate an old-QP `x0` splice.
- AS-RTI literature forms preparation about the state expected at the feedback
  instant.  It does not justify retaining an old stage clock after feedback.

## Constraints

- Keep the current production Store and publisher path unchanged.
- Do not add a lease, grace period, timeout, fallback, solver-tolerance change,
  clearance change or parameter tuning.
- Do not rebase only progress while leaving dynamic-obstacle and wall stages at
  their old timestamps.
- Exact current-world wall and obstacle proof remains mandatory in any later
  production design.
- If the suffix cannot be represented without weakening the physical contract,
  reject this design and prefer a current-state low-rate main GMPCC.

## Definition of Done

1. A pure resolver classifies elapsed time into consumed stages and a partial
   first remaining stage.
2. The deterministic mixed-origin fixture rejects `x0`-only feedback but solves
   after a complete semantic suffix rebuild.
3. Dynamic-obstacle stages and nominal path distances are shifted with the same
   temporal origin as state/input stages.
4. Invalid, exhausted, or sub-minimum remaining stages are rejected explicitly.
5. Runtime comparison remains observation-only until dynamic evidence is
   recorded.

