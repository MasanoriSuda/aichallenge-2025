# Design: proof-authoritative wall pose

The first audit arm used a strictly convex Phase-I projection before solving
the racing objective.  A direct observation arm disables only that Phase-I
step while omitting the same lag and heading pose boxes.  Both arms retain the
same rows, model, solver settings and exact proofs.

If direct succeeds, production may delete the pose boxes and keep its existing
single racing solve.  If direct fails while feasibility-first succeeds, the
production replacement must make feasibility-first globalization an explicit
part of the formulation rather than disguising it as a fallback.

No production authority changes during this measurement.
