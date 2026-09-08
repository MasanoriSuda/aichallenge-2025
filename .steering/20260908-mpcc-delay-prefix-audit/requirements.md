# Corrected-wall delay-prefix failure

The new map run finishes6laps253.198868s with0penalties but loses normal
authority at decision10699, actual7.531108m/s. That is failed integrated
acceptance. Later10700 steering-unreachable follows the explicit Emergency
publication and ledger reset; it is not the earliest cause.

First retained rejection at10699 is delay-prefix-blocked/collision with
sequence10073, published-plan clock, observation252.154994/control252.284994,
cursor0.105695. The queued-command prefix is rejected at pose
(89668.421,43169.741,-0.020), sample21of22. Full continuation and terminal
Stop are not reached. Do not change terminal reference or delay/clearance
thresholds to hide an already blocked command prefix.

Freeze current snapshot10699 and preceding fresh-worker failures10074/10081.
Compare the sealed formulation before another controller patch. Inspect
command/steering/localization/model correspondence, actual-body clearance,
and why previously certified queued commands enter the current hard reserve.
Exact prior executed-artifact availability must be verified; do not assume
that the saved fresh problem reproduces the runtime executed plan.
