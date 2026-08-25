# Requirements

## Objective

Obtain dynamic production evidence for the Pass and Return intents after
ShiftOut was promoted to the shared six-state steering-rate MPCC owner.

## Scope

- Run the committed `bfd0b4f` code without parameter changes.
- Trace each Overtake transition through canonical identity, six-state solve,
  immutable physical proof, current-world retained proof, command publication,
  and the next intent.
- Classify a failure at its earliest rejected invariant.
- Change code only if the evidence identifies a structural identity, authority,
  formulation, proof, or causal-ordering defect.

## Prohibited changes

- No wall/clearance, solver, horizon, weight, timing, or speed tuning.
- No grace period, compatibility flag, normal fallback, clamp, or authority
  switch.
- Do not restore the five-state Overtake normal owner.
- Do not modify or commit `aichallenge/result-summary.json`.

## Exit gate

- A `dev2` run reaches Pass and Return, or records the earliest structural
  reason why those phases are unreachable.
- Every reached Overtake intent reports
  `velocity-steering-progress-6state` for certified normal authority.
- No stale identity, cross-intent artifact, or uncertified normal command is
  adopted.
- Tests/build remain green after any evidence-driven correction.

