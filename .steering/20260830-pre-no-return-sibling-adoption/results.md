# Results

## Static verification

- `make autoware-build`: passed, 25 packages completed.
- `test_mpcc_overtake_sibling_adoption`: 11 tests passed.
- `test_single_authority_source_contract.py`: 83 tests passed.
- Related certified-plan, branch-bank, retained-revalidation, and architecture
  audit tests passed.
- `git diff --check`: passed.

The pure contract rejects a different epoch, target, intent, live side,
post-no-return state, hard fault, exhausted replacement budget, missing
current-world authority, and a non-stateless sibling. Tactical state mutation
is absent from retained evaluation and occurs only after the serialized
command identity joins at the single publisher.

## Dynamic verification

Run: `output/20260830-130439`, `make dev2` with explicit initial-pose and
control-mode requests for ROS domains 1 and 2.

Domain 1 entered one active encounter:

```text
Idle -> ShiftOut, target=d2, side=-1
overtake_adoption:missing-sibling-authority
ShiftOut -> Recovery, reason=actual footprint wall margin violated
```

The selected branch lost authority, but this epoch did not contain a
current-world-certified opposite sibling. The new contract therefore rejected
adoption with its intended explicit reason. There was no sibling publication,
no tactical side mutation, and no publication/tactical identity split.

This run validates the rejection path, not the successful adoption path. The
frozen counterexample in `output/20260830-120323` remains the evidence that an
exact same-epoch sibling can exist when the selected side fails. A future
natural recurrence must show `Published stateless sibling Bundle adopted`
before this mechanism is promoted from guarded production capability to a
race-quality claim.

The first attempted run, `output/20260830-125826`, never produced
`/awsim/state` or odometry because the `count`-mode simulator and Autoware
autostart handshake did not complete. It was stopped and excluded from control
evaluation. This is an independent dev-start integration issue.

## Classification

- Frozen sequence 2100: A fails while the same-epoch sibling exists, so this
  is a persistent Mission/lifecycle authority-adoption defect.
- Current dynamic run: no sibling authority existed; the subsequent wall
  failure is not evidence against the adoption contract and is not rescued by
  inventing a new fallback.
- Return authority continuity remains a separate Slice.
