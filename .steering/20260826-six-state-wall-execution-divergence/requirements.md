# Requirements

## Objective

Explain why a physically wall-certified six-state Track/Cruise horizon was
followed by actual wall contact in `output/20260825-235153`, and correct only
the earliest proven producer/model/execution contract defect.

## Expected versus observed

- Expected: a production six-state command backed by current-world wall proof
  keeps the actual vehicle footprint inside the certified corridor until the
  next receding-horizon replacement or fails closed before contact.
- Observed: recent QPs and wall proofs were healthy, but Domain 1 reached
  `wall_distance=0.000` and `e_y=-2.426 m` during Cruise; the QP rejection
  cascade came afterward.

## Constraints

- do not tune wall clearance, MPCC weights, speed, horizon or solver settings;
- do not add a new fallback, grace period, lease or controller switch;
- distinguish command transport, actuator response, vehicle-model mismatch,
  coordinate/progress mismatch and retained-proof timing;
- preserve one six-state normal authority and the existing Emergency/Recovery
  override boundary;
- preserve `aichallenge/result-summary.json`.

## Definition of Done

- command, measured state, steering response and wall-proof timelines are
  joined for the first incident;
- the earliest divergence is identified before the wall-contact symptom;
- at least two competing hypotheses are falsified;
- any production change maps one-to-one to the proven contract defect;
- static tests pass and a bounded dynamic acceptance criterion is written.
