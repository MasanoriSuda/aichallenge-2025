# Design

The d4-only run proved that clustered sensing and directional projection remove
the frozen d4 stall.  It could not produce global terminal evidence because a
production d1 entered a separate contact trap.  This run changes only that
controlled variable:

```text
d1-d4: tiny_lidar_net checkpoint + precontact_teacher
world: e2e-final, unchanged
```

This is still teacher-only.  It is not a proposed submission architecture.  If
all domains pass, the bags become candidates for run-level corrective extraction
with explicit pre-contact-teacher provenance.  If symmetric teachers still pin
one another, no further threshold patch is allowed; the teacher branch is
closed or replaced by a temporal/multi-agent oracle.
