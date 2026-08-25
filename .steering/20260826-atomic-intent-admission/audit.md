# Audit

## Observed phenomenon

In `output/20260826-055949/d2/autoware.log`:

- decision 2140 changes `Pass -> Return`;
- synchronous sequence 1551 solves and passes the immutable physical wall
  proof in 5.251 ms;
- the same decision publishes an emergency command with
  `retained-proof-unavailable`;
- decisions 2141 and 2142 repeat physical certification without acquiring
  Return production authority;
- decisions 2143 onward reach maximum iterations after emergency braking has
  changed the predecessor state.

The preceding retained telemetry already alternates
`velocity-unreachable`, `steering-unreachable` and
`dynamic-path-blocked`.  These are upstream adoption failures, not evidence
that the Return QP itself was initially infeasible.

## Root cause

`evaluate_rate_resolved_transition_admission()` ended at physical
certification and returned only a boolean and sequence.  The production owner
then rescanned the shared newest-candidate store and performed current-world
revalidation separately.  Therefore:

1. the word `certified` was incorrectly interpreted as adoption-ready;
2. the exact certified object was not part of the transition result;
3. the dynamic-world or predecessor-continuity rejection was erased from the
   atomic-admission log;
4. the tactical phase had no evidence explaining why canonical authority did
   not cross the publisher boundary.

This is an authority/adoption transaction defect.  Solver tuning, wall margin
changes and a new fallback cannot correct it.

## Implemented correction

- The transition result now owns the exact certified plan pointer.
- That exact plan receives the ordinary current-world retained evaluation in
  the same synchronous transaction.
- The production owner consumes that returned evaluation directly and no
  longer looks the transition plan up again in a mutable store.
- The log distinguishes `certified` from `joined` and reports the typed world
  reason and blocking obstacle.
- A rejected world join remains fail closed.

## Validation

- `make autoware-build`: 25 packages passed.
- complete package CTest: 51/51 passed.
- source authority contract: 56/56 passed.
- no parameter, fallback, timeout, ROS interface or evaluation-schema change.

## Remaining evidence gap

AWSIM remained in `Ready` during autonomous start attempts, so this Slice does
not claim moving acceptance.  The next manual moving run must determine
whether the first Return rejection is a dynamic blocker, reachability failure,
or a successful join.  If `certified=1,joined=0` remains, prospective Return
admission before tactical phase mutation is the next structural Slice.
