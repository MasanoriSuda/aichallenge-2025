# Results

## Verification

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest .../test_single_authority_source_contract.py -q`
  - 98 passed.
- `make autoware-build`
  - 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 59/59 test suites passed.
- Dynamic run: `output/20260831-171209`

## Dynamic evidence

The same active target/generation adopted exact certified siblings repeatedly:

| Time | Transition | Sequence | Phase |
|---|---:|---:|---|
| 1788163962.901 | -1 -> +1 | 594 | ShiftOut |
| 1788163965.200 | +1 -> -1 | 683 | ShiftOut |
| 1788163968.370 | -1 -> +1 | 812 | ShiftOut |
| 1788163972.221 | +1 -> -1 | 967 | ShiftOut |
| 1788163973.927 | -1 -> +1 | 1036 | ShiftOut |
| 1788163977.289 | +1 -> -1 | 1161 | ShiftOut |

The previous `overtake_adoption:no-return` veto did not recur. Every adoption
crossed the canonical publisher and was aligned as a current-world Bundle.
This validates the root-cause repair: Mission history no longer overrides an
exact current-world physical certificate.

## Newly exposed boundary

The run remained in ShiftOut for 15.02 seconds and 61+ metres while exact
branch feasibility moved between sides. It then entered Recovery solely due to
`same-target Mission total budget expired`.

This is not a regression of the certificate handoff. It exposes the next
persistent-Mission ownership defect:

- current-world stateless Bundles continued to publish and accelerate;
- no hard fault or global solver failure caused the abort;
- a retained Mission wall-clock budget still overrode the receding current-world
  authority and forced Recovery.

The next Slice must audit whether that budget has any authority once a
publisher-bound stateless Bundle owns active execution. Do not tune the 15 s
value; classify and remove the obsolete owner if the frozen evidence confirms
it.

## Residual risk

Branch feasibility changed sides six times. The changes were seconds apart,
not control-rate chatter, but no full ShiftOut -> Pass transition completed in
this short run. The next comparison must distinguish:

1. obsolete Mission timeout/lifecycle ownership,
2. missing stateless terminal phase transition, and
3. candidate generation that cannot maintain a completion-capable homotopy.

