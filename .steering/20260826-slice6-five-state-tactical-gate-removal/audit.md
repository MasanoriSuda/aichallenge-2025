# Audit

Baseline: `5ff812f fix(mpcc): preserve gate a execution prefix`.

The audit starts from a dynamic contradiction, not a solver parameter guess:
the same tactical snapshot is evaluated by two formulations and the older one
still owns live side/entry suppression.  The change will be accepted only when
the old producer, plan state, revalidator and cache are physically absent.

## Root cause and propagation

The tactical worker evaluated every valid side twice.  The six-state result
proved the candidate against the current physical model, but the older
five-state result still populated a separate selection, pre-entry plan and
retained cache.  A five-state `progress-regressed` result could therefore hold
entry even when the causal six-state pipeline later produced an admissible
candidate.  The visible delayed/held Overtake entry was a downstream symptom;
the root defect was two formulations owning the same tactical decision.

## Removed runtime surfaces

- five-state fresh tactical evaluator and lifecycle;
- five-state per-side solver contexts and warm starts;
- five-state pre-entry plan and retained entry-plan cache;
- five-state current-world certificate revalidator;
- duplicate left/right selection and second-formulation veto;
- production executable links to canonical execution-plan, retained-world and
  Follow canonical migration libraries.

The course-frame and control-path fingerprints required by the live six-state
physical proof moved into `mpcc_rate_resolved_physical_wall`; their inputs and
determinism are covered by a focused unit test.

## Verification

- Source-contract test: 54 passed.
- Workspace build: 25 packages succeeded.
- Package suite after the final fingerprint-test rebuild: 51/51 CTest
  targets, 1,914 tests, zero failures/errors/skips.
- Production binary: no dynamic dependency on the retired tactical Gate
  libraries.
- `git diff --check`: clean.

The only `colcon test-result` diagnostic is a pre-existing stale
`build/joycon_contract_guard/package.xml` lookup; the reported test summary is
still 1,914/0/0/0.

## Moving acceptance

Run: `output/20260826-161516` (`make dev2`, bounded moving run).

- `Idle -> ShiftOut`: 3.
- `gate=six-state-shiftout` accepted commits: 3.
- Every accepted entry had `certificate=1`, `samples=20`,
  `exact_stages=20`, and `exact_state=ey/lag/epsi/v/progress`.
- Five-state decision/selection/progress-regressed traces: 0.

This proves that the remaining five-state tactical Gate is no longer required
for moving Overtake admission.  Later `locked target stale or lost`, explicit
Emergency episodes, and runtime-overrun windows are downstream integration
quality findings.  They are not hidden with tuning or a restored fallback and
remain inputs to the final Slice 6 integration Gate.
