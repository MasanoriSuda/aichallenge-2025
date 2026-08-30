# Findings

## Observed phenomenon

The run emitted four d1 and three d2 tactical `Follow -> Overtake` transitions,
and generated six ShiftOut architecture failure snapshots.  Nevertheless, the
canonical production command never used ShiftOut, Pass or Return on any car.

## Cause versus symptom

The ShiftOut snapshots are pre-entry observation artifacts.  They can compare
homotopies before admission, but cannot prove that an active Mission lost its
authority.  Treating their solver failure as a production execution failure
would add a fallback at the wrong layer.

The production-relevant symptom is earlier: an Overtake proposal exists, but
the canonical intent remains Cruise or Follow.  Consequently the active
Overtake branch bank is cleared or remains unavailable and sibling adoption is
correctly reported as inactive execution.

## A/B/C/D results

### ShiftOut 2230

- A persistent selected side: dynamic-obstacle refinement rejected.
- B opposite stateless side: certified Bundle.
- G opposite production-direct side: certified Bundle.
- Live producer: pre-entry shadow, not production.

This demonstrates a viable alternate homotopy at that observation epoch, but
does not demonstrate a live Mission lifecycle failure.

### ShiftOut 3742 and 4709

No direct B/G branch produced a certified Bundle.  The alternate either hit an
exact wall proof or a newly detected exact dynamic overlap.  These snapshots
do not justify a clearance or solver change.

### Cruise wall 2684

- A selected negative side: wall-refinement solve rejected.
- B positive side: certified Bundle.
- Live detail: `store=accepted/adopted_side=1`.

The existing normal sibling architecture behaved as intended.

### Cruise terminal contingency 1128

Persistent, left and right direct arms all solved their QP but failed exact
dynamic proof against d2.  No implemented arm produced a certified Bundle.
The terminal-contingency failure is downstream of current candidate overlap,
not evidence that recursive Stop proof is too strict.

## Next hypothesis

An identity or lifecycle edge is lost between the tactical Overtake proposal
and canonical ShiftOut admission.  The next audit must identify whether the
Mission disappears before the pre-entry result returns, the result cannot join
the current tactical generation, Gate A rejects it, or the publisher sees a
different live intent.  No new timeout, lease, grace rule or retained path is
authorized by this finding.
