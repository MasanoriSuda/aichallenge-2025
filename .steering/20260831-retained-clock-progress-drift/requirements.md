# Requirements: retained clock/progress drift

## Objective

Classify the first production authority failure of the primary D1 vehicle after
the canonical target-tube repair. Determine whether the wall-margin Recovery is
caused by physical infeasibility, candidate generation, single-SQP, or retained
artifact clock/progress lifecycle drift.

Frozen evidence:

- baseline: `d72843c4`
- run: `output/20260831-063008`
- domain: `d1`
- last accepted retained record: decision `1789`, source sequence `931`
- first authority loss: decision `1802`
- terminal snapshot:
  `000000001802-a7b86de7e924e7ba-shiftout-side-negative-physical-proof-terminal-contingency-unavailable`

## Observed boundary

At decision 1789 the publisher clock selected stage 15 while measured progress
lagged expected progress by `-3.969626 m`; the current pose was `3.579806 m`
from the selected suffix join pose. The record was accepted through a
current-world rebase. At decision 1802 the source cursor became unavailable,
normal authority fell to Emergency Stop, and the vehicle was already only
`0.241 m` from a front wall with `e_psi=-0.402 rad`. The later
`actual footprint wall margin violated` transition is downstream.

## Constraints

- no Mission resume rule, lease, grace period, timeout, retry or fallback;
- no solver tolerance, iteration, weight or model change;
- no wall, dynamic-obstacle or vehicle clearance change;
- no production authority change before same-snapshot classification;
- do not treat the later Recovery reason as the root cause.

## Definition of Done

- Freeze the immutable failure identity and available numerical payloads.
- Compare persistent, stateless, alternate-candidate and offline feasibility
  arms with unchanged seven-state and exact physical proofs.
- Identify the earliest owner/invariant violation.
- Implement only a root-cause repair with an explicit deleted legacy path.
- Pass focused and full package tests plus a bounded dynamic run.
