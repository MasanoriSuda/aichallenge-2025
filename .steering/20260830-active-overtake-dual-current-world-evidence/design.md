# Design

## Root-cause evidence

The frozen Pass snapshot from `output/20260830-113908` produced these results
with one world and one seven-state formulation:

- selected side `+1`: SQP maximum-iteration failure at a steering-rate row;
- opposite side `-1`: solver accepted, exact wall/opponent proof accepted,
  terminal successor accepted.

At the matching live time, tactical telemetry reported
`opp_alt_side=-1` but `opp_alt_ok=0`. Production current-world generation only
called the population evaluator for `execution_side_sign`, while the existing
dual branch path was limited to Cruise/Follow. The downstream emergency Stop
therefore cannot yet be classified as physical infeasibility.

## Slice boundary

Introduce an active-Overtake evidence bank and a dual evaluator:

```text
immutable ShiftOut/Pass snapshot
       |-- side -1 population -> seven-state SQP -> exact certificates
       `-- side +1 population -> seven-state SQP -> exact certificates
                         |
                         +-> atomic evidence bank
                         `-> selected execution-side result only
```

The evaluator uses independent persistent solver contexts per side. It does
not select the opposite side and does not update tactical homotopy ownership.
If the selected side certifies, the existing certified-plan store may keep its
same-epoch sibling with it. If only the opposite side certifies, it remains in
the evidence bank and the selected-side failure continues downstream exactly
as before.

## Next decision

- Opposite Bundle arrives before Stop: adoption/lifecycle defect; connect it
  through explicit pre-no-return tactical replacement in a later Slice.
- Opposite Bundle arrives only after Stop: scheduling defect; isolate or
  cadence-control branch production before adoption.
- Neither side certifies live although replay does: live scheduling/source
  provenance defect.
- Neither side certifies in replay or live: continue C/D feasibility audit.

## Dynamic conclusion

`make dev2` produced `output/20260830-120323`. Production authority was not
changed. In episode 2, source sequence 2100 captured the decisive case:

- active intent: `ShiftOut`;
- selected side: `-1`;
- selected result: exact physical proof rejected at stage 135;
- sibling side `+1`: exact-certified in the same immutable source epoch;
- the tactical state nevertheless remained on side `-1` and later transitioned
  `ShiftOut -> FollowPrepare -> Idle`.

The sibling existed more than three seconds before the phase exited
`ShiftOut`, so this is not a late-arrival classification. The failure is not
global physical infeasibility and is not explained by a single-SQP failure on
both homotopies. Production lacks an explicit pre-no-return consumer for a
same-epoch certified sibling.

The run also exposed two independent boundaries which must not be conflated
with sibling adoption:

- episode 1 completed `ShiftOut -> Pass -> Return`, but Return later lost
  normal authority because this evidence Slice intentionally covers only
  ShiftOut/Pass;
- episode 2 also contained an epoch where both sides failed, so opposite-side
  adoption cannot be used as a universal fallback.

The observation implementation used one additional `std::async` solve for
active Overtake. Active telemetry windows reported a mean of window-average
background compute of 95.613 ms and a maximum of 471.697 ms. This proves that
the sibling must be produced by a bounded persistent executor before it is
eligible for production use; per-evaluation thread creation is evidence code,
not the final scheduling design.

The next production Slice may therefore implement sibling adoption only when:

1. both results share the exact immutable source identity;
2. the selected branch failed and the sibling is fully exact-certified;
3. tactical commit/no-return still permits the homotopy change;
4. adoption is an explicit authority transition, not an implicit fallback;
5. the dual producer is moved to bounded persistent execution.
