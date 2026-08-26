# Audit

## Baseline

Baseline commit: `9d4d63b fix(mpcc): make normal intent admission atomic`.

## Causal map

```text
tactical Mission candidate
  -> prospective branch determines ShiftOut or Pass
  -> first six-state branch proof
     -> hand-built predecessor binding omits published steering
     -> immutable snapshot rejects every attempted branch
  -> no selected Mission hint
  -> no causal six-state solver + physical proof + current-world join
  -> RateResolvedPreentryGateAProposal
  -> FSM filters proposal to ShiftOut only
  -> Direct Pass uses five-state canonical plan/resolver
  -> Mission mutates
  -> six-state normal production starts afterward
```

The first fault is not a missing Pass solver.  It is a stale formulation filter
and fallback at the FSM adoption boundary.  Moving that boundary exposed an
older upstream wiring fault shared by ShiftOut and Pass: the first pre-entry
shadow manually constructed `BoundRateResolvedTrackCruiseSubmission` without
`publication_initial_steering_rad`.  The common immutable snapshot validator
therefore rejected every attempted six-state branch before it could produce a
selected Mission hint or Gate A proposal.

The structural repair is to remove that duplicate binding implementation and
route pre-entry through the same `bind_rate_resolved_track_cruise_submission()`
owner as every other normal six-state producer.

## Reproduction

- `output/20260826-141125/d1/autoware.log`: seven pre-entry attempts, all seven
  `six-state pre-entry snapshot rejected`, zero causal proposals and zero entry
  commits.
- `output/20260826-142429/d1/autoware.log`: 23 attempts, all 23 rejected at the
  same snapshot boundary; FSM correctly logged one `proposal=0` rejection and
  never admitted a five-state Direct Pass.

## Evidence to collect

- source deletion gate for `resolve_overtake_preentry_plan()` in fresh entry;
- six-state proposal intent `Pass` produces `Pass` FSM entry;
- no `five-state-direct-pass-gate-a` production trace;
- final normal execution remains six-state or explicit Emergency;
- rejected proposals preserve the previous proven intent when current-world
  proof exists.
- first pre-entry shadow uses the common predecessor binder with both physical
  control origin and exactly published steering.

## Dynamic evidence and second root cause

After the common predecessor binder repair, Gate2 exposed a separate normal
entry bypass in `output/20260826-144042/d1/autoware.log`:

- `Idle -> ShiftOut`: 15;
- every transition used Mission generation 0;
- `unsupported-intent`: 15;
- causal six-state Gate A proposals and entry commits: 0.

The raw start-grid geometric corridor replaced the corresponding normal
`SideAssessment`, thereby discarding its complete Mission.  Global Mission
collection was also disabled during the start-grid attempt, and the FSM
explicitly excluded start-grid from `fresh_normal_mission_entry`.  The behavior
label could consequently mutate the line phase without any six-state artifact.
This was a second authority path, not a solver or parameter failure.

The repair keeps start-grid geometry as tactical preference/diagnostic data,
always gathers complete left/right Missions, and forbids a behavior-only
Overtake request from entering while the line phase is Idle.  Start-grid,
ShiftOut and Direct Pass now share the same exact six-state Gate A.

`output/20260826-145002/d1/autoware.log` confirms:

- `Idle -> ShiftOut`: 0;
- `Idle -> Pass`: 0;
- `unsupported-intent`: 0;
- generation-0 normal entry: 0.

No positive ShiftOut or Direct Pass Gate A occurred in this Gate2 geometry
because no complete Mission became available.  This closes the invalid
authority bypass but does not claim practical overtake acceptance.  The lack of
a feasible Mission remains a later strategy/quality observation and must not be
hidden by restoring the start-grid exception or tuning parameters.

## Verification

- Host source contract: 58 passed.
- Docker package test: 51/51 test targets, 1893 tests, 0 errors/failures.
- Workspace build: 25 packages succeeded.
