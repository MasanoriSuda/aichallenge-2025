# Validation

## Static

- `make autoware-build`: PASS, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: PASS, 47/47 CTest
  targets and 1,835 assertions/tests, zero errors or failures.
- Dedicated certified-plan and single-authority contract rerun: 2/2 PASS.
- Source contract confirms physical evaluation precedes certification/store,
  while no rate-resolved path reaches a command publisher.

## Dynamic shadow Gate

Run: `output/20260825-054114`, bounded `make dev2` followed by clean
`make down`.

| Domain | Certified plans | Certification reject | Store invalid/stale | Callback overrun |
|---|---:|---:|---:|---:|
| D1 | 403 | 0 | 0 / 0 | 0 |
| D2 | 814 | 0 | 0 / 0 | 0 |

Both domains ended with `available=1`, `cert_reason=none`, `last=accepted`,
`authority=shadow, selected=0`. D2 recorded one current-semantic stale
consumption in the final aggregate, but the full artifact/physical join had
zero identity reject; current semantic revalidation remains the next explicit
Gate rather than being inferred from this store.

## Result

Accepted as an evidence-ownership Slice. It removes the need to reconstruct a
certified artifact from two independently consumed latest results. It does not
extend the original world certificate in time and does not approve production
authority.

## Next blocker

Resolve an exact cursor and prove current intent, current pose and the
current-to-remaining-horizon connector against the current world. Age or the
original physical acceptance alone must not admit retained execution.
