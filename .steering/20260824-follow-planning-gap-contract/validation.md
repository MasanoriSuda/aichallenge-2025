# Validation

## Failure-first

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q --import-mode=importlib \
  aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/test_single_authority_source_contract.py
```

Before implementation: `1 failed, 3 passed`. The exact planning-gap production
wiring was absent. After implementation: `4 passed`.

## Build and tests

```bash
make autoware-build
```

Result: 25 packages built successfully. The only stderr was the existing
setuptools deprecation warning.

Focused results:

- `test_race_mpcc_foundation`: 28/28 passed;
- `test_canonical_retained_world_revalidation`: 9/9 passed.

Package result:

```text
Summary: 1728 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result` also printed the pre-existing missing
`build/joycon_contract_guard/package.xml` discovery warning; it did not create
a test failure.

`git diff --check` passed.

## Dynamic Acceptance

```bash
make dev2
# output/20260824-055552
make down
```

Against `output/20260824-051821`, Domain 1 Follow retained
`stage-gap-violation` decreased from 611 to 4 and
`canonical-follow-emergency` decreased from 651 to 17. Accepted retained
traces had minimum/maximum/average certified gaps of
3.65506/4.37812/3.98426 m, all above the unchanged 2.05 m physical hard gap.

The four residual gap rejects coincide with two target-speed-to-zero
discontinuities and are tracked separately. Overtake wall-path failures also
remain separate. This Slice meets its material-decrease gate without relaxing
physical admission.
