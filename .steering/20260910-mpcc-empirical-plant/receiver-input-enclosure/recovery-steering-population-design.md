# Recovery steering population independent of optional guidance

2026-09-13. Baseline3fa5ef2c, runtime1b478b27/single-r22/D1decision1921.
The expected bounded clear-footprint reverse population must be available when
optional Recovery MPC guidance is disabled. The actual producer couples configured
steering sampling to guidance availability, reducing selection to0/±0.25rad.
All original8m candidates collide in exactR764. R766 holds world/footprint/course/
steering limits: two-segment8m and64bounded beam8m candidates pass static/course.
R767 narrows the change to the already configured five constant magnitudes:
−0.05/−0.10rad pass original8m; −0.15 is physically clear but course rejected;
all positive and original±0.25 remain rejected. Scalar/indexed native results match.
These are Recovery kinematic comparisons, not canonical nine-state A/B/C/D or
normal authority. No third normal-family repair is proposed here.

Extract the existing heading-aligned reverse selector into a directly testable
header. Preserve original evaluator/course guard/ranking/first-result behavior.
For a fresh candidate with a clear current footprint, enumerate the existing
configured steering population even without optional guidance. Preserve committed
steering and contact-mode behavior. Remove the guidance-only restriction on that
fresh clear-footprint path; preserve forward selection and existing guidance score.
No parameter, model, footprint, distance, duration, stopping reserve, V2X, gear,
authority, solver or deadline change. No piecewise/beam/analytic actuation added.

Regression uses an exact cell crop from the sealed1921map (outside remains Unknown),
original pose/body/distance/course. Old producer must fail to select; corrected
producer must select−0.05rad. Preserve full-steering collision, −0.15course rejection,
new occupied-cell rejection, committed steering, contact policy and guidance mode.
Then source tests, native tests, full build/package and fresh committed single run.
Acceptance includes actual Recovery selection/revalidation/gear/actuation telemetry,
normal authority and callback/publication timing. A local pass is not race completion.
Rollback is the eventual implementation commit; prechange source is3fa5ef2c.

Earlier moving rear_static_blocked/hazard inputs remain absent and are not explained
by1921. VehicleStart/laps, physical observation, current velocity constraints,
r80actual publication timing and M4–M6 remain open. No safety stop is deleted.

Validation: R768 old selector fails one new scene test, other68pass; R769native249/source118pass.
R770full-grid old3reject/current11select−0.05; original three native results remain exact.
Build138all26/sourceunchanged; package125 Summary: 2651 tests, 0 errors, 0 failures, 0 skipped.
[Sealed evidence](recovery-steering-population-validation.json). Fresh committed single-r23next.
