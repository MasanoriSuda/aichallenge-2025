# Follow asynchronous canonical producer design

## Ownership split

```text
40 Hz live callback
  current Follow contract + immutable scene snapshot
       | submit_latest (non-blocking)
       v
Follow canonical worker
  build -> solve -> physical certificate -> immutable canonical plan
       | latest-only mailbox
       v
40 Hz live callback
  typed result identity
  -> current-world Follow retained proof
  -> canonical selector + exact actuation
  -> shadow telemetry only
```

The worker proves that its plan was valid for its snapshot. The live proof establishes whether the
remaining part is executable now. These are different certificates and neither substitutes for the
other.

## Implementation slices

### A. Shared Follow lifecycle refactor

Move the dedicated solver context, warm-start identity/epoch and canonical plan store behind one
shared lifecycle object. Tactical snapshots share that lifecycle rather than creating divergent
solver/store state. Runtime behavior remains synchronous in this sub-slice.

### B. Typed async result boundary

Add a Follow-specific latest-only mailbox containing sequence, context epoch, snapshot decision/time,
target/intent provenance, result status and an immutable canonical plan. Test ordering and stale
publication without ROS.

### C. Worker connection in shadow

Deep-snapshot mutable model/reference/wall/target inputs, run fresh Follow production in the worker,
and remove the live synchronous call. A completed plan can enter the shared store only after current
identity checks. The live callback then uses the existing current-world retained proof.

### D. Dynamic gate

Compare callback p95/p99/max, overruns, worker submitted/replaced/completed, result age, fresh-plan
adoption and current-world command reconstruction. Do not promote authority in this steering.

## Deletion boundary

When C is connected, delete the synchronous fresh Follow solve from `get_control()`. Do not leave it
as a fallback. The only live Follow shadow computation is current-world proof/selection of an
immutable canonical plan.
