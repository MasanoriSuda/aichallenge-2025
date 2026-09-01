# Design

The current policy has three distance-only states: clear, zero acceleration and
full brake.  At zero speed with clearance between 1.5 and 3.0 m it commands
zero forever, although an Ackermann vehicle needs motion to change lateral
position.

The candidate instead converts frontal clearance to a safe speed by inverting:

```text
usable_distance = front_distance - minimum_clearance
usable_distance = v_safe * reaction_time
                + v_safe^2 / (2 * effective_deceleration)
```

Acceleration is limited by `gain * (v_safe - v)` and the existing braking
command.  Inside the hard minimum clearance it still commands full braking.
Outside it, a stopped vehicle receives only the bounded acceleration implied by
the remaining clearance; no dedicated creep flag, timeout or recovery state is
added.

The first audit evaluates effective decelerations 1, 2 and 3 m/s2.  This is a
model-sensitivity comparison, not permission to publish a stronger brake
command.  It also compares an unbounded envelope with a slow-zone-gated
variant.  The gated variant is active only inside the existing 3.0 m exposure
boundary, so it cannot reinterpret distant course walls as a new obstacle
class.
