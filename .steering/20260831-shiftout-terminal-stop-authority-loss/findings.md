# Findings: ShiftOut terminal-Stop authority loss

## Observed phenomenon

A progressing ShiftOut selected a certified Stop while the target remained
about 21 m ahead.  The following Emergency Stop, full deceleration, target loss
and Recovery were effects of normal authority loss, not its cause.

## Causal chain

1. Frozen Mission/DP geometry aged while the kart progressed into a hard curve.
2. The last published normal artifact approached the end of its 20-stage
   horizon.
3. The active dual producer solved the committed branch and alternate branch as
   one joined background task.
4. Alternate work delayed current-world primary Store admission.
5. The old normal artifact could no longer join the current vehicle state.
6. Its certified terminal Stop was selected as designed.
7. With the normal ledger interrupted, the next cycle had no published normal
   artifact and Emergency Stop became effective.

## Root cause and evidence

Root cause: cross-branch scheduling coupling in the active Overtake producer.

Evidence:

- persistent A failed while stateless B-left and production G-left passed all
  exact proofs on the same immutable snapshot;
- B-left included a certified terminal Stop suffix, so the accepted result was
  not an unsafe forward-only trajectory;
- runtime artifact 771 was at stage 18/20 before the Stop transition;
- production code waited on `negative_executor->wait()` before Store admission;
- the current-world B-left solve took about 0.16 seconds, enough for the old
  artifact to exhaust during a hard-curve state transition.

## Existing patches

The terminal Stop bridge, exact wall proof and current-world retained checks
behaved correctly and exposed the gap.  Weakening them would hide the defect.
Legacy viability demotion and Mission preservation increase the time spent with
old geometry, but no new timeout or Mission-resume rule is required for this
failure.

## Implemented change

- removed the opposite-branch `wait()` from active Overtake production;
- made the committed side the sole primary Store producer;
- moved the opposite side to an observation-only sibling executor;
- retained same-epoch branch-bank evidence without giving it command authority;
- renamed the executor from `negative` to `sibling` to match its responsibility;
- updated the single-authority source contract test to prohibit reintroducing
  the cross-branch wait.

## Removed/organized behavior

The producer no longer needs a temporary negative result, negative wait result,
or `replace_pair()` rendezvous before publishing the selected branch.  This
removes one lifecycle join rather than adding another fallback.

## Remaining concerns

- The primary solve itself may still exceed one 25 ms control period; the
  latest-only producer measured 45--100 ms pipelines during the dynamic run.
- A selected branch that is genuinely infeasible is still rejected.  Automatic
  opposite-side adoption remains governed by existing no-return policy and is
  not changed by this Slice.
- The first dynamic Overtake exposed a separate authority defect: a stateless
  sibling Bundle changed the Mission side from -1 to +1, while execution-source
  projection still held the old side identity.  That produced
  `side-mismatch`, then stale/missing solved-source rejection and Stop.  This is
  not evidence against the primary/sibling scheduling fix; it is the next
  frozen lifecycle boundary to audit.
- Frozen Mission geometry remains tactical state.  A later Slice may remove
  more retained geometry, but only after this scheduling defect is dynamically
  verified.

## Dynamic verification

Run: `output/20260831-160811/d1/autoware.log` (`make dev2`).

- `active-overtake-primary` certified and stored the selected side while the
  sibling executor was both `accepted` and `busy`; therefore primary Store
  admission no longer waits for sibling completion.
- The background evidence contained
  `selected_certified=1`, `selected_bank=accepted`, `store=accepted` and
  `sibling_submit=busy` in the same primary result.
- Build completed for all 25 packages and the focused source-contract suite
  passed 97 tests.
- The run is not an Overtake-quality acceptance: at log line 516 an existing
  stateless sibling adoption changed side, lines 519--520 rejected the joined
  execution identity, and later ShiftOut entered Recovery after target loss.
  The next Slice must freeze that adoption boundary before changing code.

## Next-run acceptance

- `active-overtake-primary` appears in producer evidence;
- selected-side certified results reach the Store without sibling completion;
- sibling executor busy/failure does not change selected normal authority;
- no terminal Stop occurs while a same-snapshot primary bundle is certified;
- ShiftOut either advances to Pass or reports a selected-side physical proof
  rejection, rather than `steering-unreachable` after sibling delay;
- callback and background p95/max are recorded for comparison with the frozen
  run.

The first three scheduling criteria passed.  Full phase completion did not;
the failure classification changed from cross-branch wait to side-identity
handoff, providing a bounded next root-cause Slice rather than a parameter
adjustment.
