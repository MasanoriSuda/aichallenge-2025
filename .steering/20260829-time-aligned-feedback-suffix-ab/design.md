# Design: time-aligned feedback suffix A/B

## Causal model

The prepared candidate owns absolute stage boundaries

`t0, t1, ..., tN`.

At feedback time `tf`, the new problem must start at the latest measured state,
then target the first old boundary strictly after `tf`.  If `tf` lies inside
stage `k`, stages before `k` are consumed and the first duration becomes
`t(k+1) - tf`; later durations remain unchanged.

This keeps wall and opponent predictions attached to their original absolute
future times.  It intentionally shortens the remaining horizon instead of
inventing an unpredicted tail.

## Rebuilt semantic snapshot

- `request.initial_state`: latest seven-state physical observation, projected to
  the existing five semantic state fields plus steering/yaw-response origins.
- `states[0]`: exact latest state bounds/reference.
- `states[1..]`: old states at the remaining absolute stage boundaries.
- `inputs[0]`: old active input with only its duration shortened.
- `inputs[1..]`: remaining old inputs unchanged.
- `dynamic_obstacle_stages`: shifted by the same consumed stage count.
- `nominal_path_distance_m`: interpolated at `tf`, sliced and rebased to zero.
- physical wall profile and course-progress origin: unchanged; they are
  continuous functions in the original local course frame.
- problem context: new stage geometry/horizon identity, resealed before solve.

## A/B arms

- A: current latest-state feedback (`x0` replacement in the old final QP).
- B: full adapter/refinement solve from the time-aligned semantic suffix.

B is deliberately more expensive at first.  The experiment asks whether the
problem formulation is correct before optimizing derivative reuse.

## Promotion gate

No authority promotion in this Slice.  B must first show materially higher
feasible/certified rate without stale-world acceptance.  If it remains too slow
or infeasible, stop and select the upper-style current-state main GMPCC instead
of adding another connector patch.

