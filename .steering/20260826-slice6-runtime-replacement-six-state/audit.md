# Audit

## Baseline

Baseline commit: `447d239 fix(mpcc): require six state gate for all overtake entry`.

## Authority map

```text
active tactical left/right assessment
  -> prospective six-state branch proof
  -> causal current-world join
  -> six-state Mission Gate A proposal (generation current + 1)

parallel legacy path
  -> five-state OvertakeExecutionArtifact OR geometric Mission only
  -> runtime replacement contracts
  -> freeze_selected_overtake_mission()
  -> Mission generation changes
  -> six-state normal producer tries to solve afterward
```

The parallel legacy path is the first causal fault.  Solver rejection after a
replacement is a downstream symptom.

## Reachable callers

The replacement boundary is shared by early-side rescue, runtime wall replan,
dynamic Mission wait, MPCC-lite same/cross-side replan, opponent-side replan,
safe-separation soft abort, tactical revalidation and pass-horizon refresh.
Only two MPCC-lite callers carry a five-state artifact; the others can mutate
from geometry alone.  All must converge on one causal six-state Gate A.

## Implemented authority boundary

Every caller now submits only a requested geometric Mission and the current
`RateResolvedMissionGateAProposal`.  The shared replacement boundary rejects
before mutation unless all of the following are exact:

- prospective generation is current generation + 1;
- target id and target obstacle generation;
- requested side and proposal side;
- phase-derived execution intent;
- `VelocitySteeringProgress6State` formulation;
- certified solver/wall/current-world plan identity.

The proposal's immutable Mission is used for the replacement contracts and is
the only Mission that can be frozen.  A missing or mismatched proof retains the
currently proven Mission.  `OvertakeExecutionArtifact` and the complete
`resolve_overtake_preentry_plan()` five-state resolver surface, including its
seven tests, were physically removed.

## Verification

- Failure-first source contract failed on the old
  `OvertakeExecutionArtifact`, then passed after removal: 58 tests passed.
- `make autoware-build`: passed, 25 packages built.
- Full `multi_purpose_mpc_ros` package test: 51/51 targets,
  1886 tests, 0 errors, 0 failures, 0 skipped.  The seven-test reduction is the
  intentional deletion of the retired five-state resolver tests.
- Moving Gate2 run: `output/20260826-150956`.

At `d1/autoware.log:448` the causal proposal was complete and joined to the
current world (`solver=solved`, `physical=accepted`, `mission=1`,
`proposal_identity=1`, `gate_a_proposal=1`).  At lines 450-451 the controller
entered `Idle -> ShiftOut` and accepted `gate=six-state-shiftout`, generation 1.
The next decisions published matching certified six-state ShiftOut solutions.

The run did not exercise a runtime Mission replacement, so positive replacement
adoption remains dynamically unobserved.  This is not replaced by a claim from
the source contract: the contract proves that an unproved replacement cannot
mutate state, while a future run must still exercise a successful replacement.

## Separate defect exposed by acceptance

About 0.8 seconds after ShiftOut entry, the active six-state normal solve began
reaching 4000 iterations.  The first failed row was the progress-rate input box
(`stage=0/element=2`), with a large primal/dual residual.  Certified retained
plans then became stale, one normal submission reported incomplete source
context, and ShiftOut eventually entered Recovery after the target became
stale.  No runtime replacement log occurred before this failure.

This is downstream six-state formulation/lifecycle debt, not evidence that the
removed five-state replacement authority should return.  It is carried into
the next Slice as a root-cause item; no solver parameter or fallback was added
here.
