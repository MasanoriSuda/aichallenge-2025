# Audit

Baseline: `5957aab fix(mpcc): gate runtime replacement with six state proof`.

This Slice starts from evidence, not from the downstream OSQP symptom.  The
accepted ShiftOut proposal and first retained rejection are recorded in
`design.md`.

MCAP inspection falsified the initial observation-age hypothesis.  The exact
source stamp used by retained validation was current.  Source inspection then
found the upstream ownership break: Gate A retains a six-state
`CertifiedPlan`, but the committed Mission previously received only the
tactical candidate.  The log confirms the result as `certificate=0` with a
31-sample tactical execution path immediately after a physically accepted
six-state Gate A.

The repair binds the exact Gate A trajectory into the Mission before mutation
and prevents the tactical path from overwriting it during freeze.  No solver,
margin, tolerance, lease or fallback parameter changes are part of this Slice.

## Verification

- Failure-first source-contract test: 58/58 passed.
- Workspace build: 25 packages succeeded.
- Package test: 51/51 targets, 1,886 tests, zero failure/error/skip.
- Moving acceptance: `output/20260826-153933`.

The moving run admitted two ShiftOut Missions.  Both admissions reported
`certificate=1`, `samples=20`, and `exact_stages=20`; no accepted entry used a
zero certificate or the former 31-sample tactical DP path.  The first episode
published 41 joined production commands before the next independent failure.

The first subsequent authority loss was
`retained=dynamic-path-blocked`, followed later by an optimized-horizon wall
failure and Recovery.  It is not the old Gate-A-to-FSM certificate loss.  This
new evidence is carried into the next Slice; this Slice does not relax the wall
proof or add a retention exception.
