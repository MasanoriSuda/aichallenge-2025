# Design

## Evidence ownership

Extend the pure minimum-speed admission request with an optional
`certified_execution_minimum_speed_mps` value. When finite and non-negative it
owns the predicted-speed side of the comparison. Otherwise the existing
Mission prediction remains the fallback.

At the asynchronous entry join, derive this value only from a complete exact
physical execution certificate:

1. require `physical_execution_certificate_valid`;
2. validate the exact trajectory shape;
3. take the minimum of its five-state velocity sequence;
4. pass it atomically with the existing Mission snapshot requirement.

The later current-world revalidation remains mandatory before any FSM commit.
Thus the change fixes a proof-source mismatch; it does not turn an old or
wall-invalid solution into authority.

## Rejected alternatives

- Lowering the 8 m reserve: hides the join defect and reopens the squeeze
  failure the gate was introduced to prevent.
- Changing Follow distance: couples normal following behavior to a temporary
  migration defect.
- Relaxing minimum-speed tolerance: still judges the wrong formulation.
- Adding a lease or fallback: creates another authority path instead of fixing
  the existing production join.
