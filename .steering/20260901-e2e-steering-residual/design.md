# Design

## Root cause addressed

The previous full-network candidate learned the corrective subset but changed
normal behavior and still trapped one peer.  Freezing only the final layer kept
normal behavior but could not encode the new correction.  The conflict came
from asking one parameter set to retain the accepted racing policy and learn a
sparse, structurally different contact-avoidance policy.

## Data contract

For every scan the admitted base checkpoint supplies one base steering.  Both
teachers receive that exact base value.  Because production executes
`base + learned residual` and does not execute `LidarGapTeacher`, the residual
baseline must be the raw frozen base command.  Dataset generation stores:

- `steers.npy`: successor teacher's absolute command (existing contract)
- `base_steers.npy`: frozen production TinyLidarNet command
- `reference_steers.npy`: historical teacher command, diagnostic only
- `steering_deltas.npy`: successor minus frozen base, the runtime target
- `reference_steering_deltas.npy`: historical teacher minus base, diagnostic
- `successor_upgrade_deltas.npy`: successor minus historical teacher, diagnostic

The runtime composition identity is recomputed and checked rather than inferred
from an absolute label.  This avoids the rejected contract where a
successor-minus-reference target was incorrectly added directly to the raw
base policy.  Train and validation use separate source domains.

## Model and loss

The residual network consumes the same normalized 750-point LiDAR scan and
outputs one bounded steering delta.  Its final layer starts at exactly zero, so
the untrained composition is exactly the production policy.

Loss is per-sample Smooth L1.  Material corrections receive a bounded weight;
the much larger zero/small-delta population remains in every epoch as normal
anchors.  Metrics are always split into:

- material correction (`abs(delta) >= 0.02 rad`)
- zero/small anchor
- all samples

## Promotion order

1. held-out residual validation;
2. independent normal-run perturbation audit;
3. optional runtime override, residual disabled by default;
4. single vehicle, NPC, then four-peer closed-loop gates.

No threshold, longitudinal safety or production checkpoint is changed to make
the residual appear successful.

## Closed-loop outcome

The bounded residual runtime and audit path are valid, but no learned candidate
is promoted.  The selected diagnostic candidate passed single-vehicle and NPC
motion gates, then wedged d1 and d2 in the four-peer world from about 118 s.
Both domains continued to command positive acceleration.

Pre-failure DAgger showed that nearby single-frame LiDAR observations required
opposite correction signs.  Longer optimization, equal sequence sampling and
both-side training either averaged the correction to zero or leaked into held-
out anchors.  This is an architecture/input observability limit for the current
stateless residual, not evidence for changing wall or braking thresholds.

The production checkpoint and default launch remain unchanged.  A temporal or
otherwise stateful ML policy is a separate follow-up Slice.
