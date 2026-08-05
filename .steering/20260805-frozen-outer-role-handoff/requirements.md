# Requirements

## Goal

Prevent a committed outside pass from becoming an inside pass after the
reference-path curvature changes sign. The complete first outer-role handoff
must be geometrically admitted before ShiftOut and then executed from the
frozen mission instead of re-deriving a lateral goal from target jitter.

## Evidence

`output/20260805-222815/d1/autoware.log` contains a scheduled handoff with
`side=-1->1` and window `[16.51, 24.51] m`, but the only runtime attempt was
rejected with `no wall-feasible same-side separated goal`.

The target was still 3.34 m ahead and the bodies were already separated. The
runtime planner nevertheless required the new outer goal to retain 1.5 m
lateral center separation from the target. That requirement is inappropriate
for a lateral crossing performed longitudinally behind the target and causes
the admitted mission to be replaced by a different, infeasible problem.

## Required behavior

- Select and freeze the opposite outer-role lateral goal at mission admission.
- Validate the transition ramp, remaining Pass path, Return path, wall bounds,
  steering curvature and lateral acceleration before admitting ShiftOut.
- During execution, use the frozen transition goal; do not move it because the
  target lateral estimate changes.
- Permit lateral center-line crossing only while the target remains ahead by
  the configured longitudinal intrusion guard and current/predicted footprints
  are separated.
- Preserve live hard gates for stale V2X, target discontinuity, wall contact,
  acceleration infeasibility and loss of longitudinal clearance.
- If the frozen transition is infeasible, reject the complete mission before
  ShiftOut instead of entering Pass and later falling into Recovery.

## Constraints

- Keep ROS topics, messages, launch structure and evaluation interfaces intact.
- Do not relax global wall or gap parameters.
- Do not change Recovery or braking behavior in this task.
- Scope the change to the first scheduled curvature-role reversal; subsequent
  reversals continue to use the existing bounded rolling mechanism.

