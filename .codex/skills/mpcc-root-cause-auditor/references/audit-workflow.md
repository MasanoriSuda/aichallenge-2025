# MPCC root-cause audit workflow

## 1. Fix the evidence boundary

Record:

- branch and baseline commit;
- working-tree changes to preserve;
- run ID and Domain;
- first abnormal timestamp/decision ID;
- last known normal timestamp/decision ID;
- whether evidence is source-only, unit test, replay, simulation, SIL, HIL, or vehicle.

Do not combine logs produced by different commits/configurations as one causal timeline.

## 2. Describe the phenomenon before causes

Separate:

- expected behavior;
- actual behavior;
- where the problem is first observable;
- where the visible consequence occurs;
- which input, state, solve, certificate, and output remain normal.

The visible failure and the producer may be far apart.

## 3. Build the authority graph

Trace:

```text
observation
-> target/intent
-> candidate/homotopy
-> corridor
-> admission
-> problem construction
-> solver
-> nonlinear/physical certificate
-> selection
-> post-processing
-> fallback/recovery
-> publish
```

For each node record owner, context identity, bypass, stale adoption condition, failure destination,
and whether the destination changes formulation.

## 4. Build hypotheses

For each hypothesis include:

| Field | Required content |
|---|---|
| Hypothesis | The upstream contract suspected to fail |
| Supporting evidence | Source/history/log/test evidence |
| Falsifier | Observation that would disprove it |
| Needed observation | Earliest missing variable/certificate |
| Confidence | High/Medium/Low with reason |

Prefer competing hypotheses. Do not start with OSQP, warm start, wall validator, or planner as the
assumed culprit.

## 5. Inspect patch history

Use `rg`, `git blame`, `git log -S`, and `git log -G` for relevant symbols and terms:

```text
fallback retry cooldown hold grace reentry rescue continuation
suppress clamp lease prearm backoff handoff legacy schema-ready
```

Record introduction reason, suppressed symptom, intended invariant, dependencies, and deletion gate.
Opaque `wip` history is `Unknown`; inspect the diff or reproduce the old failure before deletion.

## 6. Check core invariants

At minimum check:

- selected implies solved and physically certified;
- selected solution, executed trajectory, certificate and command share one fingerprint;
- lateral and longitudinal normal commands share one solution;
- async context matches observation, target, geometry, horizon, bounds and cost schema;
- compared objectives share the same schema;
- stage geometry is consistent across dynamics, corridor and physical validation;
- incompatible warm starts are invalidated;
- schema-only/shadow-only candidates cannot execute;
- solver failure does not switch to another normal formulation;
- Recovery records the upstream failure identity.

## 7. Build causal trees

For each failure, write the chain from the earliest violation to the visible symptom. Label every
edge Root, Contributor, Mask, Detection gap, or Recovery. If two independent violations are required,
record a minimal causal cut set rather than forcing one cause.

## 8. Compare fixes

Compare at least:

- repair producer and delete masks;
- retain current structure but improve checks;
- migrate the relevant vertical slice to the canonical formulation.

Evaluate authority count, branches/configuration added and removed, warm-start compatibility,
certificate consistency, replayability, timing, migration risk, and legacy deletion gate.

Reject a proposal whose main effect is threshold tuning or an additional exceptional branch unless
the user explicitly approves it as temporary and its removal test is defined.

## 8a. Architecture escape-hatch

Before a third implementation Slice in the same failure family, or when the
package `AGENTS.md` trigger fires, stop production changes and demote the
current architecture to one candidate. Seal one immutable, replay-ready
snapshot and compare:

- A: persistent Mission pipeline plus the current canonical SQP;
- B: stateless receding ManeuverBundle plus the same SQP;
- C: an independently generated rough path plus the same refinement;
- D: a bounded offline multi-SQP or nonlinear feasibility solve.

The compared methods must share world/problem fingerprint, state, reference,
wall map, peer prediction, physical model and hard constraints. Record every
outcome in the central experiment registry. The comparison is observation-only
and cannot publish or change production authority.

Classify `all failed` as physical infeasibility only with an explicit bounded
certificate. Otherwise classify it as `Unknown` and improve evidence or search
coverage before changing production.

## 9. Implementation gate

Before an approved implementation slice, report:

1. files to change;
2. failing test/replay to add first;
3. root producer to change;
4. mask/bypass to delete;
5. new branches/configuration count;
6. remaining legacy authority;
7. rollback commit.

Then implement only that slice, verify focused tests, package tests/build, available replay, and
authority/fingerprint telemetry. Report both added and deleted production paths.

## 10. Audit report structure

```markdown
# Executive summary
# Observed phenomenon
# Current authority graph
# Hypotheses and falsifiers
# Patch ledger
# Invariant table
# Failure causal trees
# Root causes versus masks
# Candidate fixes and tradeoffs
# Recommended migration slice
# Pre-fix replay/test
# Deletion plan
# Unknowns and measurement plan
```
