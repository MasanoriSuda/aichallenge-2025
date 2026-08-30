# Requirements

## Baseline

- Source baseline: `7dc46d3f fix(mpcc): align return intent with full horizon proof`.
- Dynamic evidence: `output/20260830-101331`, Domain 1/2.
- Upper-system comparison: `.steering/ano`.

## Observed failure

The run contains 38 logged canonical Emergency decisions; 21 explicitly report
`rate-resolved authority unavailable`.  At decision 5475 the selected Cruise
artifact sequence 4547 loses current-world terminal Stop proof while the new
proposal is `progress-lift-rejected`.  The retained diagnostic reports no
same-epoch alternate (`normal_branches=...seq:0/...missing-plan`), so external
Emergency becomes the only authority.

The producer already solved both current-world homotopies.  However its global
branch bank is replaced by every newer source epoch, including an empty pair.
By the time an actually selected or published artifact needs its sibling, the
bank describes a different, newer world and has legitimately discarded the
old pair.  The selected artifact and the sibling that was certified with it
therefore have different lifecycles.

## Root-cause hypothesis

Candidate-set generation and candidate execution ownership are split.  The
selected certified plan crosses the Store and publisher, but its same-epoch
certified sibling remains only in a latest-world diagnostic bank.  This loses
one half of the immutable candidate set at the producer/Store boundary.

Emergency is a downstream symptom.  The first violated invariant is:

> Every selected normal-avoidance artifact retains its same-source certified
> sibling until that selected artifact is superseded at the corresponding
> candidate, published-Bundle, or executed-plan lifecycle boundary.

## Objective

1. Bind the selected and opposite certified plans from one source epoch into
   one immutable Store transaction.
2. Preserve the sibling independently for candidate, published-Bundle source,
   and executed-plan entries.
3. If ordinary candidate/published/executed revalidation all fail, evaluate
   only siblings associated with those exact entries.
4. Require the sibling to pass the existing current-world wall, dynamic,
   actuation and recursive terminal Stop proof before gaining authority.
5. Keep the latest-world branch bank observation-only; do not use it as the
   execution lifecycle owner.

## Constraints

- Do not change MPCC weights, solver settings, horizon, clearance, vehicle
  limits or control frequency.
- Do not add a timeout, lease, grace period, retry, fallback or resume rule.
- Do not publish an old sibling by age or source identity alone.
- Do not change Emergency authority or any ROS/evaluation interface.
- A sibling must be the opposite dynamic-obstacle side from the selected plan,
  with the same sequence, snapshot time and source problem epoch.
- Sibling adoption must traverse the unchanged production revalidation chain.
- No Mission geometry or persistent path sample authorizes a sibling.

## Acceptance

- Unit tests reject mixed-epoch, same-side and invalid sibling pairs.
- Store tests prove candidate, published and executed entries retain the
  sibling which was atomically paired with the selected plan.
- A newer empty global branch-bank epoch does not erase an executed entry's
  sibling.
- Retained evaluation tries the associated sibling after ordinary evidence is
  exhausted and records exact reason/sequence/side.
- Focused and complete package tests pass; `make autoware-build` passes.
- Dynamic trial observes associated-sibling inspection or adoption in the
  frozen family, and does not publish stale or uncertified authority.
