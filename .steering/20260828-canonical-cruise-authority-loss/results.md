# Results

## Static evidence

- `make autoware-build`: completed successfully.
- package CTest: 52/52 passed.
- The source-time audit was observation-only and had no publication path.

## Dynamic evidence

Bounded run: `output/20260829-001005`.

Domain 1 initially published candidate sequence 310 and retained it through
61 accepted current-world cycles.  Before authority loss, 30 of 38 consumed
worker results solved and the certified candidate store advanced to sequence
384.  The executed sequence nevertheless remained 310.

At decision 979:

- executed sequence 310: `progress-lift-rejected`, then horizon exhausted;
- candidate sequence 384 at cursor zero: `steering-unreachable`;
- candidate 384 at source time `0.125 s`: still `steering-unreachable`;
- requested steering: `0.087963 rad`;
- reachable interval from the exact wire predecessor:
  `[0.222855, 0.274468] rad`;
- production response: explicit canonical Emergency.

This rejects physical-wall and simple cursor-age hypotheses.  The QP and
physical proof completed, but the actuator followed the old certified plan
while the new plan solved and the two command prefixes diverged.  Treating the
unpublished prefix as executed would violate command provenance.

## Classification

Offline/current-world solve evidence exists while live adoption fails:
**scheduling/lifecycle defect**.

The next experiment compares the failed async adoption with a same-cycle
current-world solve of the identical seven-state problem.  No solver, wall,
clearance, timeout, lease or fallback parameter is changed.
