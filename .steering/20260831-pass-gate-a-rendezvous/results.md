# Results: Pass Gate-A rendezvous

## Observed phenomenon

In `output/20260831-100351/d2` episode 2, ShiftOut reached its lateral
completion boundary while the current callback had no Pass Gate-A proposal.
Certified Pass proposals existed in earlier and later callbacks, including
decision 1808 (`solver=solved`, `physical=accepted`, `dynamic=valid/clear`,
`authority_ready=1`), but the instantaneous completion predicate was false
when that late proposal arrived. The controller remained in ShiftOut for about
8.38 seconds and eventually reported `locked target stale or lost`.

## Root cause

The phase consumer required two independent asynchronous events—physical
ShiftOut completion and a current-world certified Pass result—to occur in one
25 ms callback. `V2XBehaviorOutput` intentionally does not retain Gate-A
proposals, while the physical completion predicate can become false after the
boundary. The handoff therefore had no causal rendezvous.

## Implemented change

- Added an identity-scoped monotonic ShiftOut boundary resolver.
- It retains only target ID, Mission generation, side and the observed fact.
- It resets on phase exit, new ShiftOut entry, side replan or any identity
  mismatch.
- Pass still requires a proposal freshly joined and certified in the consuming
  callback; no path, corridor or certificate is retained.
- Admission logging now distinguishes instantaneous completion from the
  identity-scoped boundary fact.

No Mission resume rule, timeout, lease, grace period, fallback, solver setting,
wall margin, clearance or velocity policy changed.

## Verification

- `make autoware-build`: 25 packages succeeded.
- complete package suite: 59/59 CTest targets, zero failures.
- first bounded run `output/20260831-102150`: the relevant rendezvous was not
  reached; D1/D2 failed earlier through independent wall paths.
- second bounded run `output/20260831-102636`: D2 logged
  `completion=1/1, proposal=0`, retained ShiftOut, then 28 ms later consumed a
  fresh certified proposal and completed
  `ShiftOut -> Pass -> Return -> Idle`.
- A later episode used another side and did not reuse the first boundary fact;
  it independently entered DynamicMissionWait for an unavailable corridor.

## Remaining independent failures

The two runs still contain `actual footprint wall margin violated`,
`Pass entry physical gate has no valid current-side prefix` and
`live overtake corridor unavailable`. They occur before or after the repaired
rendezvous and remain separate frozen failure families.
