# Mission lifecycle A/B comparison result

## Executive summary

The experiment did not reach a same-snapshot solver comparison between A and
B.  It instead exposed an earlier architecture boundary: while an Overtake
encounter was active, the current pipeline could not make the persistent
Mission candidate and a fresh receding candidate independently available at
the same time.

This is not sufficient to classify the failure as `A fails, B succeeds`.
It is sufficient to reject another lease, grace, timeout or resolver-precedence
patch.  The next B must be produced independently from the current world as an
InteractionBundle rather than sourced from the persistent Mission lifecycle.

## Evidence boundary

- Branch: `develop_july`
- Frozen production baseline: `0287d1930bfdd7b89c36a25cb9f75bd76800d762`
- Evidence type: `make dev2` simulation log
- Domain with observed interaction: d1
- Runs:
  - `output/20260828-073117`
  - `output/20260828-074011`
  - `output/20260828-074712`
- Production authority: unchanged; all comparison records used
  `authority=observation-only/selected=0`

## Observed phenomenon

Expected:

- one immutable active Overtake world can provide A, the retained complete
  Mission, and B, a freshly rebuilt same-homotopy receding candidate;
- both candidates can then enter the same seven-state solver and physical
  proof pipeline.

Observed:

- the first live hook did not expose non-comparable outcomes;
- after adding explicit wait telemetry, run `20260828-074011` recorded 26
  `persistent-mission-unavailable` and 28
  `receding-candidate-unavailable` outcomes;
- the persistent arm was then adapted only inside the private observation copy
  to use the already-valid frozen runtime Mission geometry;
- run `20260828-074712` subsequently recorded 18
  `receding-candidate-unavailable` outcomes over two ShiftOut encounters;
- no `MPCC_AB` record reached two attempted solver arms.

Representative evidence:

- run `20260828-074011`, d1, sequence 96, phase Pass, target d2, side +1:
  persistent arm was unavailable under candidate-generation labels despite an
  active runtime Mission;
- run `20260828-074712`, d1, sequences 44--50 and 131--141, phase ShiftOut,
  target d2, side -1: the persistent runtime geometry was available but no
  fresh receding candidate existed.

## Causal chain

1. The tactical worker evaluates the active persistent Mission world.
2. A is represented by the frozen runtime Mission.
3. B is not independently rebuilt from that same world; it is read from the
   optional `mpcc_receding_mission` product of the existing tactical pipeline.
4. During active ShiftOut, that optional product is absent.
5. The comparison stops before problem construction and solver execution.
6. The ordinary production resolver hides this gap because the complete
   persistent Mission has precedence and remains the only candidate presented
   to the solver.

## Root cause versus masks

- Earliest violated invariant: an active interaction snapshot must be able to
  produce each architecture candidate independently of the candidate it is
  being compared against.
- Producer: the existing receding arm is lifecycle-dependent optional state,
  not a stateless current-world candidate generator.
- Contributor: complete Mission precedence makes the missing B invisible in
  normal operation.
- Detection gap: ordinary branch-selection logs report only the selected
  candidate and cannot distinguish absent B from A precedence.
- Mask: retained Mission execution permits the encounter to continue without
  proving that a current-world alternative exists.
- Recovery behavior: Emergency and Recovery remain downstream safety actions;
  neither is classified as this experiment's root cause.

## Classification

The A/B solver result is `inconclusive` because no comparable pair was solved.
The architecture escape-hatch result is `candidate comparison blocked by
lifecycle coupling`.

The next experiment must implement B as a stateless InteractionBundle producer
with no dependency on frozen Mission path, phase-transition path, Mission age
or retained candidate availability.

## Production cleanup

The bounded live A/B hook and its unused candidate-isolation API were removed
after evidence capture.  This restores the controller source exactly to the
frozen production baseline and avoids leaving first-encounter CPU contention
in production.

No solver parameter, wall clearance, fallback, lease, grace period, command
publisher or production authority was changed.

## Remaining C/D work

- B: independent stateless InteractionBundle plus the common seven-state SQP;
- C: independently generated spline, polynomial or lattice candidate plus the
  same refinement, only if A/B cannot produce a feasible candidate;
- D: bounded offline multi-SQP or nonlinear feasibility solve, only if A/B/C
  fail on the same replay-ready snapshot.

`all failed` must remain `Unknown` unless D provides an explicit bounded
physical infeasibility certificate.

