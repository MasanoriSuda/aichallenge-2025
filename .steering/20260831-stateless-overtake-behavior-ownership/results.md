# Results

## Frozen comparison

`output/20260831-112650/d1` and the diagnostic rerun
`output/20260831-115922/d1` establish the same A/B classification:

- A, persistent Mission plus seven-state SQP: the replacement side's legacy
  candidate is rejected by wall-clamp preflight;
- B, stateless current-world Bundle plus the same seven-state SQP: exact
  physical/dynamic/terminal proof succeeds and the command is published;
- C and D are not limiting because B succeeds.

The diagnostic run adopted sequence 598/generation 1 and then reported all
target/hard guards clear, while Behavior repeatedly demoted to Follow. The
only missing source fact was the independent generation-only publication
ledger. The canonical publisher had already validated and committed the exact
sibling token, so this was duplicate lifecycle ownership rather than physical
infeasibility or solver failure.

## Structural correction

Sibling adoption now records an immutable publication identity:

```text
{stateless_sibling_source_generation, stateless_sibling_source_sequence}
```

Behavior accepts that identity or a validated frozen Mission as mutually
exclusive source types. It does not restore retired Mission geometry, inherit
its body-clear deadline, add a lease/fallback, or relax any proof.

## Verification

- `make autoware-build`: 25/25 packages succeeded.
- Explicit full test-target build completed.
- Package `ctest --output-on-failure`: 59/59 passed in 25.25 s.
- Dynamic run: `output/20260831-122218/d1`.

Dynamic evidence:

- `Idle -> ShiftOut` occurred for target `d2`, generation 1;
- sequence 896 was published and adopted from side -1 to +1;
- subsequent Behavior diagnostics retained
  `stateless=1/seq=896/gen=1` despite independent Mission candidate rejection;
- the former immediate `Overtake -> Follow` contradiction did not recur;
- later release occurred only after `locked target stale or lost`.

## Residual failure family

The same run contains a separate canonical Stop retention / normal-authority
unavailability interval that reduces actual speed to zero before target loss
and Recovery. It is not caused by stateless source ownership and must be
frozen and compared independently before any change.
