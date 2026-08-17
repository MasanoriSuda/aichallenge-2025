# Design

## Root cause

The previous repair validated each stage with path yaw, then moved only the
failing stage toward the base line. Since footprint yaw depends on `d'(s)`, a
single-stage correction can steepen the adjacent slope. The final validator
then reports a physical failure before a coupled, smooth correction is tried.

The validator also used a forward difference for most samples, while the
executed `target_epsi` profile is generated from the current lateral state and
successive backward differences.

## Changes

1. Add a pure recovery-footprint primitive that validates a complete Frenet
   lateral profile. Stage zero uses `current_lateral -> target[0]`; later
   stages use `target[i-1] -> target[i]`.
2. Separate scalar bound/reachability preparation from static-map validation.
3. Replace the final point-wise repair with a bounded whole-profile search:
   retain as much of the optimized profile as possible while blending toward
   either a current-side hold or a smooth centre-line return.
4. Re-run lateral-acceleration validation on each complete candidate profile.
5. Populate the physical failure stage index and exhaust all bounded
   speed/wall-reserve candidates before declaring a hard failure.
6. Retain an already revalidated last-feasible prefix for a future-profile
   miss; actual current wall contact, emergency and solver guards remain hard.

## Safety

- Occupied, unknown and out-of-map cells remain infeasible.
- The vehicle footprint and configured hard wall margin are not reduced.
- Profile contraction is bounded, deterministic and centreward/current-side.
- No cross-side switch is introduced by this repair.
