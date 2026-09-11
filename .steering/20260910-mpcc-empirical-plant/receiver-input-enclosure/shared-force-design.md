# Shared wheel-force Jacobian

2026-09-11 JST. Production baseline/rollback f3b444f2; documentation checkpoint
05f4847e. This slice continues the authorised M1–M6 implementation.

The earliest remaining invariant failure is dev2-r29 D1 decision831: publication
completed30ms after the nominal epoch, outside the existing25ms proof window.
The publisher detects this; there is no integrated acceptance. Its exact proof
input was not captured. Historical sealed r27/r26/r24 inputs independently
reproduce expensive proof evaluation, without substituting for the missing r29
input. Their rejected and accepted outcomes are retained.

The current seven-component interval automatic differentiation repeatedly
evaluates wheel-force expressions. For a fixed tire interval, each wheel force
is affine in forward/lateral speed, yaw rate and drive plus rolling acceleration.
R197 assembles those coefficient intervals and their steering partials once
inside one native midpoint map. Both stages have identical tire intervals because
body_increment changes only the six body coordinates. Each use checks the model
pointer and exact tire endpoints. There is no reuse across native steps or requests.

Only the derivative evaluation changes. Value intervals still evaluate the
original shared kernel verbatim. Clamp, reverse correction, rolling, tire slew,
all discontinuity branches, integration steps, input populations, full rest,
wall/peer margins and normal/async/publication authority remain unchanged.
Outward interval operations assemble the coefficients and chain all seven
derivative components. The existing natural/centered enclosure intersection
remains. No prior authority path is added or replaced.

R198 independently differentiates the original kernel at100 decimal digits:
400 boxes,4800 points,38400 value checks and260736 smooth-branch derivative
checks pass;12 discontinuous boxes check values only. R199 preserves the unsafe
r28 fixed programme rejection and126 native wall-contact checks; all six
programmes' body/corner inclusion checks pass. R200–202 retain the15-case native
suite outcomes except r24 D2 decision990, now accepted through complete rest,
Stop materialization and join with peer clearance+0.0004877070832474m.
That is enclosure precision, not a smaller safety margin. R18 actual history gap
still rejects with prediction reason5. Long r27 rejection still costs about70ms;
offline improvements do not establish the runtime deadline.

Before committing: format the implementation without changing expressions; add
a permanent independent high-precision kernel comparison and coefficient-context
negative checks; build all packages and run the complete MPCC package suite;
replay the native fixtures against the rebuilt production library. Seal sources,
commands, results, rejected r193–194 reuse hypotheses and r195–202 comparisons in
the experiment registry. Then run fresh committed dev2-r30 and continue M4–M6.

- [x] Bound the numerical hypothesis and compare isolated implementations.
- [x] Independent high-precision and native input/corner checks.
- [x] Production implementation and permanent meaningful regressions.
- [x] Buildr62, package testsr51 and rebuilt-library replayr203–206.
- [x] Seal evidence, update specification/registry and local commit7c283cc7.
- [ ] Fresh live acceptance; remaining M4–M6.


Buildr61/r62:26packages passed. R50 reported two expectations in one obsolete
coarse-current classification test; r51 passes2462records in63groups. The frozen
coarse checkpoint still rejects the axis separator. The current tighter tube
passes both separators with identical body/corner bounds; a concretely occupied
initial footprint rejects both. Rebuilt productionr203–206 matches every
non-timing field of the isolated15-case suite and retains126unsafe native contacts.
See [sealed evidence](shared-force-evidence.json). Live deadline acceptance is open.


Dev2-r30rejects beforeReady atD1645/D2662finalpublicationwindow. Numerical
improvements remain validated but insufficient for integrated acceptance.
[Next observation slice](publication-observation-design.md) captures the exact
finalguardinput for causal repair; M4–M6remains open.
