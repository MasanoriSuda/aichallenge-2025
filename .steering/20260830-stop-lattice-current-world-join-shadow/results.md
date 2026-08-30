# Results

## Static verification

- `make autoware-build`: 25 packages passed.
- `multi_purpose_mpc_ros` package tests: 2281 tests, 0 failures.
- single-authority source contract: 92 tests passed.
- `git diff --check`: passed.

The source contract proves that the observer calls the existing current-world
evaluator but cannot write the certified-plan Store, mark execution, create
pending actuation or publish a command.  Diagnostic plans are accepted only
when their identity matches the currently published Overtake source.

## Dynamic run

Run: `output/20260830-235803`, Domain 1.

Episode 1 entered ShiftOut at decision 1579.  Stop lattice evaluations ran in
parallel and produced accepted observation-only plans.  At decision 1689 the
ordinary selected plan lost authority:

```text
intent=shiftout
normal=terminal-contingency-unavailable
selected normal source=1053
terminal Stop wall proof=collision
```

At the same exact current-world boundary, diagnostic source 990 evaluated as:

```text
joined=1
reason=accepted
authority=shadow
selected=0
```

Production nevertheless emitted the external canonical emergency Stop at
0 m/s.  A fresh normal source briefly recovered at decision 1690, but the
external Stop was selected again at decision 1691 and retained from decision
1692 onward.  The episode later entered Recovery after an actual-footprint
wall-margin violation.

The observation itself caused one callback overrun at decision 1689
(57.847 ms total, 54.093 ms in MPC).  This is acceptable only for the shadow
experiment; production must not repeat the current-world solve after the
ordinary join has already failed.  A production edge must evaluate each
candidate once and atomically select the accepted result.

## Classification

**A fails, alternate certified artifact joins: lifecycle/source-replacement
defect.**

This is not physical infeasibility: an immutable lattice artifact passed the
unchanged current-world velocity, steering, static-wall, dynamic-obstacle and
terminal-contingency contracts at the failure decision.  It is not evidence
for changing wall margin, solver tolerance, Mission lease or timeout.

The later `actual footprint wall margin violated` Recovery is a distinct
candidate/geometry quality issue.  It must not be hidden by the authority
bridge change.

## Next Slice

Promote the already accepted current-world lattice evaluation as an alternate
normal source only at the ordinary ShiftOut/Pass authority-loss boundary.
The promotion must:

- use the same exact current-world evaluation result, without a second solve;
- preserve one canonical normal publisher and immutable identity;
- atomically select ordinary or lattice authority before external Stop;
- retire the observation-only duplicate edge in the same Slice;
- keep external Stop when both sources reject;
- leave wall, solver and timing parameters unchanged.
