# Design

The physical wall refinement owns four state buckets at stages 1..N:

- lateral;
- lag;
- heading;
- progress.

The prior decision-2473 audit restored only heading because that was the
minimal witness for that one snapshot. Corpus decision 1566 proves this is not
the architectural boundary: its minimum feasible relaxation spans the full
pose bucket.

The generalized Phase-I problem therefore restores those four state boxes to
their pre-refinement semantic bounds while retaining the selected explicit
progress-wall rows, swept-footprint rows, inputs, steering limits, velocities
and exact affine dynamics. This relaxed result remains non-certifiable. It is
only a tangent used to rebuild new physical buckets and the unchanged full QP.

If the explicit wall envelope itself is infeasible, as in decision 1161, the
arm must remain rejected. It may not delete wall rows or manufacture another
side inside this solver arm.
