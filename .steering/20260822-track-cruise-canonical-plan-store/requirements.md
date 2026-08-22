# Track/Cruise canonical execution-plan store

## Baseline

- Branch: `develop_july`
- Baseline commit: `409d3a3`
- Preserve `aichallenge/result-summary.json`.

## Root cause addressed

The authority selector can now require a plan ID and current-decision proof, but there is no typed
object proving that the ID names a complete five-state prediction and its exact three-input control
sequence. A caller could still synthesize metadata from an OSQP warm start, a first-stage proposal
or an already-consumed plan.

Before Track/Cruise authority promotion, the runtime needs one atomic unit containing:

- original problem and solution certificate identity;
- all `N + 1` five-state predictions;
- all `N` `[acceleration, curvature, virtual progress speed]` inputs;
- each input's execution duration;
- monotonic plan identity and solve time.

## Required invariants

- Partial plans are never stored.
- An older asynchronous result cannot replace a newer plan.
- Cursor advancement never clamps to and repeats the final stage.
- A retained candidate is produced only from the actual remaining sequence.
- Current-pose wall and obstacle proof must match the plan ID, cursor and current decision.
- Invalid replacement leaves the previously accepted plan unchanged.

## Non-scope

- No connection to `mpc_controller_cpp.cpp`.
- No Track/Cruise authority promotion.
- No changes to controller weights, margins or published commands.
- No simulation behavior change.

## Exit gate

- Deterministic tests cover incomplete, stale, future, expired and exhausted plans.
- Atomic replacement and compare-by-plan-ID clearing are tested.
- Candidate construction rejects mismatched or physically uncertified revalidation.
- Build and complete test suite pass.
