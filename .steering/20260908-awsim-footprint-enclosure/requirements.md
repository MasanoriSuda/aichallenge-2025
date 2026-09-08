# Enclose the measured local AWSIM body

Static asset/transform extraction establishes a solid body forward maximum
1.614851201m, left0.767926137m, right0.767158703m. The current declared
front1.49m plus0.05m margin still omits0.074851m of the body. This is an
independent model-containment defect; the trial3 contact was on the right
side and is not attributed to front undercoverage.

Use measured nominal extents rounded outward to1mm: front1.615m, both sides
0.768m. Keep the already conservative rear0.51m and all margins unchanged.
Update the two active simulation configurations and the corresponding
controller defaults. Do not change steering, speed, model wheelbase,
tracking margin, map, target prediction, or certificate acceptance.

The existing physical-world producer derives peer radius by subtracting ego
half width from the planner's combined lateral distance. Therefore that
distance must also be recalibrated to0.768+0.768=1.536m in both configurations
and the corresponding planner default. Otherwise an ego-width correction
silently shrinks the peer model. The regression reproduces that coupling.
Prediction times, margins and kinematics stay fixed. Full peer longitudinal
enclosure/reference-point correspondence is a separate open audit.

A fixture contains the independently extracted 2D convex hull and source
asset hashes. Both configured nominal footprints must contain every vertex,
without consuming safety/tracking reserve to hide a nominal mismatch.
Failing test precedes the configuration repair. This establishes ego-body
enclosure only; static map coverage and peer geometry remain separate.

Rollback baseline remains a8b968ac plus the pre-slice GNSS-calibrated run's
participant patch and binaries. No actual vehicle operation is authorized.
