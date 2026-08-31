# Results: normal authority-loss snapshot

## Implemented observation boundary

- Extracted one current-world interaction snapshot builder shared by the
  existing terminal-contingency observer and the new final-authority observer.
- The builder seals the serialized predecessor, seven-state request, exact
  physical wall world, current control prefix and V2X replay world before a
  file can be submitted.
- The new observer is called after Stop-suffix, Gate-A and previous-intent
  joins, and before missing authority is converted to external Emergency Stop.
- Snapshot persistence remains on the bounded background observation worker.
  It does not solve, publish, retain or write a production candidate store.

## Static validation

- `make autoware-build`: passed, 25 packages.
- `test_single_authority_source_contract.py`: 101/101 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 59/59 passed.
- Existing terminal-contingency recording uses the same sealed builder and
  remains observation-only.

## Dynamic evidence

Bounded run: `output/20260831-192221`.

The run reproduced the first final Follow authority loss at decision 1187:

- requested/effective intent: Follow;
- preceding published intent: Cruise;
- current Follow reason: `intent-mismatch`;
- preceding Cruise rejoin: `continuation-rejected`;
- actual speed: 3.77 m/s;
- front distance: 3.25 m;
- published symptom: external Emergency Stop at -3.0 m/s2.

The observation-only recorder wrote:

`output/20260831-192221/d1/mpcc_architecture_snapshots/000000001187-bcdc90f34f8f6aec-follow-side-neutral-physical-proof-normal-authority-unavailable/snapshot.yaml`

The run was stopped immediately after the required evidence was obtained.

## Frozen A/B/C/D comparison

All ordinary comparison arms failed inside the linearized dynamic-obstacle QP:

| Arm | Outcome |
|---|---|
| persistent A | maximum iterations, coupled lateral/progress row |
| stateless left/right B | maximum iterations, dynamic-obstacle rows |
| rough lattice left/right C | all 210 candidates rejected |
| bounded offline left/right D | three-SQP attempts still rejected |

This does **not** establish physical infeasibility.  The rejected iterate was
not a valid predicted state, but the immutable warm-start trajectory from the
same exact-QP snapshot passed the physical nonlinear oracle and every exact
proof:

- terminal progress: 16.5672 m;
- terminal velocity: 3.29412 m/s;
- minimum lateral reserve: 1.37058 m;
- result: accepted ManeuverBundle.

## Classification

The frozen state is physically viable, but the linearized dynamic-obstacle QP
cannot represent or recover its already viable warm trajectory.  This is a
**model/certificate mismatch at the dynamic-obstacle refinement boundary**,
with the Cruise-to-Follow lifecycle loss as the downstream Stop trigger.

The next Slice must compare each generated Follow dynamic-obstacle row against
the accepted nonlinear trajectory.  No timeout, tolerance, clearance, solver
setting or production authority change is justified by this evidence.
