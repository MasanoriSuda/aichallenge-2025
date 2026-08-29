# Requirements

## Objective

Remove the duplicate Pass-entry certificate owner which rejects an executing,
published, exactly wall-certified seven-state ShiftOut artifact after projecting
it through the legacy `OvertakeLineHorizon` model.

## Evidence boundary

- Baseline: `5d6542e9`
- Dynamic evidence: `output/20260830-024504/d1/autoware.log`
- The same ShiftOut artifacts (`1635`, then `1772`) repeatedly passed retained
  current-world wall and opponent proof and owned the published command.
- At the ShiftOut boundary, the legacy projection nevertheless reported
  `execution horizon requires wall clamp`, held for 1.03 s / 3.05 m, invalidated
  Mission generation 2, and entered `DynamicMissionWait`.
- Runtime hard-wall fault was false. This is therefore not evidence of physical
  infeasibility; it is a certificate/model ownership mismatch.

## Constraints

- Do not change wall clearance, solver settings, timing leases, or speed policy.
- Do not weaken actual wall contact, unavailable wall sample, current-world
  opponent proof, or terminal Stop-contingency proof.
- Do not allow an unpublished or identity-mismatched artifact to authorize the
  transition.
- Atomic intent admission remains responsible for retaining the previously
  published ShiftOut authority until a current-world Pass artifact joins.
- The legacy projection remains observation/preflight only when no canonical
  published execution artifact exists; it must not recertify canonical output.

## Definition of Done

- A matching published canonical ShiftOut artifact is recognized as the sole
  normal Pass-entry execution certificate owner.
- Its exact trajectory is not sent through `evaluate_overtake_line_horizon()`.
- Missing/rejected canonical identity does not fall through to a weaker source.
- Runtime hard faults continue to prevent unsafe execution through the existing
  hard guard.
- Focused tests, full package build/tests, and a dynamic `make dev2` run pass.
- Dynamic evidence shows either a Pass transition or a newly classified blocker
  upstream/downstream of the removed duplicate certificate.
