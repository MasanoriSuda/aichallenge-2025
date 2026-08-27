# MPCC single-authority engineering policy

This file applies to `multi_purpose_mpc_ros`. It supplements the repository-root `AGENTS.md`.

## Target architecture

The migration target is one canonical MPCC formulation owning all normal racing commands:

- Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return, and Rejoin are intents and constraints,
  not separate normal controllers.
- Lateral and longitudinal commands come from the same certified MPCC prediction.
- Emergency stop may override the normal command.
- Stuck, contact, gear, and reverse Recovery remains a separate supervisor.
- Legacy MPC, three-state progress fallback, and low-speed direct normal control are temporary
  migration paths with explicit deletion gates.

Do not interpret "single MPCC" as putting target selection, homotopy selection, map processing,
emergency stop, or reverse Recovery inside one solver. Those components may remain separate, but they
must not independently emit a normal racing command.

## Architecture escape-hatch

Single certified normal authority is an invariant. A specific persistent
Mission representation, candidate generator, MPCC formulation,
convexification schedule, or solver backend is not an invariant.

Stop production changes and run a same-snapshot architecture comparison before
the third patch in one failure family, or earlier when any of the following is
observed:

- two root-cause hypotheses for the same scene have been falsified;
- ShiftOut is repeatedly demonstrated but Pass or Return has no positive
  dynamic acceptance;
- another resume, reconnect, replenish, retry, timeout, lease, grace or
  fallback rule is being proposed;
- a numerical solve and exact physical certificate repeatedly disagree;
- fresh/retained authority disappears repeatedly for the same immutable
  world/problem fingerprint.

The minimum comparison keeps production authority unchanged and evaluates the
same immutable failure snapshot with:

1. persistent Mission plus the current seven-state SQP;
2. stateless receding ManeuverBundle plus the same SQP;
3. a rough spline, polynomial or lattice candidate plus the same seven-state
   refinement;
4. a bounded offline multi-SQP or nonlinear feasibility solve.

Do not call all-method failure physical infeasibility unless an explicit
bounded physical infeasibility certificate exists. A local optimizer failing
to find a solution is `Unknown`.

Record accepted, rejected and inconclusive comparisons in the central MPCC
experiment registry. Do not repeat a rejected experiment unless its recorded
revisit condition is satisfied.

## Root-cause-first default

Controller, planner, solver, wall/corridor, authority, handoff, and recovery regressions begin in
`AUDIT_ONLY` mode unless the user explicitly approves an implementation slice.

Before changing production code or runtime config, establish:

1. a deterministic failing test/replay, or a precise plan to obtain one;
2. the earliest violated invariant;
3. the producer of the invalid state;
4. the downstream fallback/guard that masks or amplifies it;
5. the obsolete branch/configuration the fix should remove;
6. objective acceptance and rollback criteria.

If evidence is insufficient, add or propose observation at the earliest uncertain boundary. Do not
patch the last visible error.

## Changes that are not root-cause fixes by themselves

Do not use the following as the sole fix without explicit user approval, causal evidence, an expiry
condition, and a deletion test:

- increasing solver iterations or loosening tolerances;
- changing weights, clearances, margins, timeouts, hysteresis, cooldowns, or rates;
- adding fallback, retry, rescue, hold, grace, clamp, suppression, lease, or feature flags;
- retaining old and new normal authorities indefinitely "for safety";
- suppressing/downgrading diagnostics;
- changing tests to accept the failing behavior;
- catching an invalid state downstream instead of preventing its creation.

Parameter tuning starts only after the relevant structural migration slice passes.

## Authority invariants

- A normal candidate may execute only when it is solved, finite, constraint-valid, and physically
  certified.
- The selected solution, executed trajectory, physical certificate, and final normal command must
  share one immutable problem fingerprint.
- Lateral and longitudinal normal commands must come from the same solution ID.
- Async results require full context compatibility; age or target ID alone is insufficient.
- Objectives may be compared only under the same horizon, state/input schema, weights, constraints,
  and terminal semantics.
- Intent, formulation, horizon, geometry, or schema changes invalidate incompatible warm starts.
- Schema-only, shadow-only, unattempted, and uncertified candidates are never executable.
- A normal solve failure may use only a bounded, same-formulation, same-context last-certified
  solution, followed by Emergency Stop. Do not transfer to another normal controller.
- Recovery entry must retain the upstream normal decision/failure identity.

## Steering and implementation slices

Use one `.steering/YYYYMMDD-title/` per approved vertical slice. Record:

- repaired invariant and earliest violation;
- scope and explicit non-scope;
- failing replay/test;
- files to change;
- branches/configuration to delete;
- newly added branches/configuration (normally zero);
- remaining legacy authorities;
- safety/timing acceptance;
- rollback commit.

Implementation order:

1. demonstrate the pre-fix failure;
2. repair the root producer;
3. remove the downstream mask/bypass made obsolete;
4. run focused tests, package tests, build, and available replay;
5. verify authority/fingerprint telemetry;
6. review the diff for new exceptional paths.

A slice is incomplete if it adds a new normal authority but deletes none, unless it is an explicitly
approved shadow-only measurement slice with a named deletion milestone.

## Evidence and reporting

- Code claims use `file:line`.
- History claims use commit IDs or `git log -S/-G` evidence.
- Runtime claims use run ID, Domain, timestamps/decision IDs, and log/rosbag evidence.
- Label unsupported conclusions `Unknown` or `Hypothesis`.
- Separate root cause, contributing cause, mask, detection gap, and Recovery behavior.
- Report simulation, SIL, HIL, and physical-vehicle evidence separately. Current AWSIM evidence is not
  real-vehicle validation.

For a root-cause audit, use the repository skill `mpcc-root-cause-auditor`.
