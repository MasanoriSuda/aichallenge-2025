# Actual longitudinal command application

Continue after1c4f377e. Full MPCC acceptance remains open. The explicit Odometry
source-to-now repair passes native/build/single and exactly matches live current
poses; dev2still rejects firstD1decision930. No production change selected here.

Use `output/20260909-stop-viability-dev2-r1` exclusively for the new boundary.
Actual normal374 ->Stop386 at928 ->normal374 at929 ->Emergency930. Keep actual
publication374and finalinspected386 evidence/clock separate; runtime also reports
an independentStop from374dynamic-rejected. Do not infer wrong source selection
is the cause without a separate current-world revalidation of actual374.

Published wire brake: source9.859999779, receipt1788931444.3009799, acceleration-3;
normal resumes9.884999779/receipt1788931444.3144498/+1.32959628105. Emergency
9.909999778/receipt1788931444.350290/-3. The25ms source interval is a publication,
not an application observation. Bag receipt is not Unity or controller receipt.

Local immutable VehicleRosInput IL: Ackermann callback queues latestLongitudinalInput;
UpdateQueuedLongitudinalInput uses UnityTime.time and a minimum elapsed interval
max(.01,1/actuationLongitudinalUpdateRateHz). At acceptance it assigns
lastAppliedLongitudinalTime=now and clears pending. YAML rate10Hz. Thus exact
application is frame/arrival dependent, not a known fixed phase or0.13s transport.
Separate internal emergency topic (immediate) from MPCC's Ackermann brake output.
Do not tune delay/gain/rate/hold or claim application solely from publication.

Decode raw same-run VelocityReport/IMU/steering/clock and commands. Determine
whether the short Stop produced a measurable brake response, and whether current
prediction/application model accounts for observed changes. Preserve real sensor
noise/coordinates, vehicle geometry/dynamics/config and source/bag clocks. Future
samples are scoring only, never predictor inputs. Obtain missing minimum evidence
if phase cannot be inferred; no additional normal authority or weakened proof.
Before repair require earliest failed invariant and native/replay reproduction.
Any formulation patch requires bounded same-world comparison with Unknown for
all-method failure absent physical infeasibility proof. Finish appropriate
build/tests/replays/runtime/spec/registry/local commit and full remaining gates.
