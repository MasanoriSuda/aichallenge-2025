# Results

## Observed failure chain

`output/20260830-144538` showed the following causal chain:

1. The active Pass producer solved and certified both sides at newer epochs.
2. The Overtake branch bank accepted both plans.
3. The production Store rejected `replace_pair()` with `invalid-plan`.
4. Sequence 1379 remained the last Store candidate/executed evidence.
5. Its expected progress moved more than 1.5 m ahead of the vehicle.
6. Retained proof changed to `progress-lift-rejected`.
7. Normal authority became unavailable, emergency Stop was published and
   stuck recovery followed.

The old run contains six active-Overtake `store=invalid-plan` records.

## Root cause

The active Overtake dual producer correctly supplied a ShiftOut/Pass sibling
pair to the common Store. The Store's sibling validator, however, allowed only
Cruise/Follow contexts. Any non-null Overtake sibling therefore invalidated an
otherwise certified selected plan. A one-sided solve was accidentally more
admissible than a two-sided solve.

## Change

The Store now has one sealed sibling relation with two typed families:

- Cruise/Follow: opposite `dynamic_obstacle_side_sign`;
- ShiftOut/Pass: opposite `execution_side_sign` and
  `dynamic_obstacle_side_sign`.

Sequence, snapshot time, intent and every remaining sealed context field must
match. Return and unrelated intents remain single-plan entries. The same
relation protects candidate replacement, publication, execution promotion and
associated-sibling lookup.

No solver, Mission, timing, fallback, tolerance or clearance changed.

## Static verification

- `make autoware-build`: 25 packages passed.
- certified Store tests: 24/24 passed.
- New tests cover Overtake atomic candidate/publication and rejection of
  wrong epoch, mixed intent and same-side pairs.
- Existing normal sibling lifecycle tests remain green.
- single-authority source contract: passed.

## Dynamic verification

Run: `output/20260830-145407`

- `Idle -> ShiftOut` at log line 678.
- `ShiftOut -> Pass` at line 784.
- Six sampled active-Overtake summaries reported `store=accepted`.
- No active-Overtake summary reported `store=invalid-plan`.
- A second episode also reached `Idle -> ShiftOut -> Pass` before shutdown.

This directly exercises the repaired producer/Store path under both ShiftOut
and Pass execution.

## Separate remaining failure

The first Pass later entered Recovery with
`SafeSeparation aborted: invalid input`. At that time the selected-side QP had
failed at its dynamic-obstacle lateral row while the same-epoch sibling was
certified. This is downstream of the repaired Store admission and is not
hidden by this change.

## Conclusion

The stale retained Pass episode was caused by a producer/Store type-contract
defect, not by proof age or progress tolerance. The repair admits current
two-sided evidence and removes the observed `invalid-plan` failure mode. The
next Slice should trace why a certified sibling is not usable at the later
SafeSeparation transition without adding another grace or fallback.
