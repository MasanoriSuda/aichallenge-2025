# Results

## Root-cause evidence

The production change is based on the frozen architecture snapshot from
`output/20260828-174825`:

- persistent Follow A: solver rejected;
- stateless positive-side B: certified;
- stateless negative-side B: certified and retained more terminal progress and
  terminal velocity.

This classifies the observed Follow stop as a tactical/candidate ownership
defect (`A fails, B succeeds`), not physical infeasibility, clearance tuning or
solver tolerance.

## Static verification

- `make autoware-build`: passed (25 packages).
- focused MPCC/authority tests: 3/3 passed.
- full `multi_purpose_mpc_ros` CTest: 52/52 passed.
- `git diff --check`: passed.

The tests preserve the following invariants:

- Follow semantic identity stays side-free;
- no second worker, publisher, store or authority is introduced;
- persistent Follow is evaluated first;
- side candidates use independent solver contexts;
- only the selected certified candidate may enter the canonical store;
- current-world retained validation and the serialized publication join remain
  mandatory.

## Dynamic Gate attempt

`make dev2` was attempted in `output/20260828-181750`. It did not reach a
controller evaluation:

- d1 observed only `state=spawned`;
- d2 did not observe an `/awsim/state` transition;
- odometry stopped after startup and became stale;
- the normal MPCC worker counters remained at zero;
- explicit control-mode requests to Domains 1 and 2 did not advance AWSIM.

Therefore this run is classified as an AWSIM/dev2 startup-handshake blocker,
not evidence for or against the Follow change. No controller fallback, timeout,
lease, solver or clearance change was added to mask it. Dynamic acceptance
remains required on the next runnable dev2 session.

## Remaining evidence

On the next valid dev2 run, confirm:

1. a failed persistent Follow solve is followed by a `follow-escape-*`
   candidate result;
2. only a certified selected candidate reaches the candidate store;
3. retained current-world validation accepts that artifact before publication;
4. stopped/slow target handling no longer falls directly to Emergency;
5. no stale-side, wall-proof or dynamic-proof regression occurs.
