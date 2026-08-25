# Design

## Initial hypotheses

1. The six-state adapter applies a strict physical-bound inset to virtual
   progress speed even though it is an internal timing variable, turning the
   valid zero-progress boundary into an artificial positive lower bound.
2. The first progress-state interval plus `theta[k+1] = theta[k] + dt*vtheta`
   requires zero or negative `vtheta`; the positive inset makes the QP truly
   infeasible.
3. The physical feasible set is nonempty, but the steering-state extension
   worsens conditioning and row 254 is only the largest residual of a
   maximum-iteration solve.
4. Warm-start transport seeds an invalid negative `vtheta` and amplifies, but
   does not create, the failure.

No production correction is selected until a failure-first fixture or joined
runtime diagnostic distinguishes these cases.

## Diagnostic design

`analyze_first_stage_input_feasibility()` intersects the declared stage-zero
input box with every stage-one state bound whose affine row depends only on the
selected input. It does not project a solution, clamp a command, alter the QP or
claim feasibility of the whole coupled problem. An empty interval is therefore
a sufficient proof of a producer contradiction; a nonempty interval only
falsifies this narrow hypothesis.

Rejected solves also decode their worst physical row into kind, stage and
element. The diagnostic is emitted only on an existing rejection warning, so it
does not create periodic log volume.

## Runtime conclusion

`output/20260825-235153` falsifies hypotheses 1 and 2 as the incident root:

- all observed first-`vtheta` diagnostics were
  `separable/feasible`, with declared and implied intervals equal;
- the worst row varied between row 253 (stage-zero steering rate) and row 254
  (stage-zero virtual-progress speed);
- the failures appeared under ordinary Cruise as well as Overtake pre-entry;
- the last healthy one-second Track/Cruise window solved 81/81 QPs in 50
  iterations and physically accepted 65 current-semantic horizons;
- at `1787669558.966791396`, before the sustained rejection cascade, the
  vehicle was already at `wall_distance=0.000`, `e_y=-2.426 m`, speed
  `8.21 m/s`, with no relevant V2X target;
- the later rejected solve at `1787669561.689...` therefore describes a QP
  initialized from an already abnormal wall/off-course state.

The row residual is a downstream symptom in this run. Warm-start, virtual-
progress inset and solver tuning remain unproven and are not changed. The next
causal question is why a physically accepted Cruise horizon did not match the
executed vehicle response before wall contact.
