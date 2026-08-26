# Design

## Retired island

```text
VelocityProgress5State enum
  -> canonical_execution_plan
  -> canonical_retained_revalidation
  -> canonical_retained_world_revalidation
  -> canonical_execution_plan_adapter
  -> follow_canonical_async / canonical_normal_async alias
```

This island is built and tested but has no production consumer.  Keeping it
creates a second normal-authority vocabulary and an easy rollback-by-reconnect
path.

## Removal boundary

Delete the island, its CMake targets and tests.  Remove the five-state enum
from the common execution contract so only six-state or explicit exceptional
bypass identities can be represented.  Delete the five-state-only shadow
warm-start contract and unused controller wall-proof helpers.

Do not delete `ExactPhysicalExecutionTrajectory`, stage geometry, target tube,
or generic branch ranking.  Those types are formulation-neutral physical data
still consumed by the six-state pipeline even where historical comments call
them five-state.
