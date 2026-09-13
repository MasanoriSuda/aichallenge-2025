# Current interval observation run and first vehicle Start

2026-09-13. single-r25 / 38a562ee / D1, build140/package127, original DLL,
120hostsec. Vehicle states spawned/grounded/ready/start; gameStart also observed.
3321normal sends,4573callbacks,19pre-send refusals with no same-decision normal
publication, zero actual window violations. Last4560/job4555/source7776/index2:
nominal104.714997764, before/after104.729997659, deadline104.739997764.
Public peak2.588056564mps at78.644998242;clock200Hz,velocity/steering28.571,
IMU20,pose49.971,trajectory1,commands39.265. No completed lap in this bounded window.

No current interval observation occurred;R797 correctly reports not-observed.
No Recovery maneuver selected or Rejoin executed. The one-shot observer never
ran and cannot be credited with the changed outcome across independent worlds.
The late interval producer defect remains Unknown; neither clearance nor a fix
is inferred. All original source/binaries and protected artifacts verified/restored.

One callback3646 exceeds25ms:26.727315ms, main current proof24.633799ms with
24.594274msCPU, normal join24.846ms, initialization1.334ms. Source5966/job3643/
index0 actually sends at81.759998172 within nominal81.739998233 and deadline
81.764998233. This is a computational callback overrun with a valid actual send,
not an actual late-send incident. No exact3646current request captured; do not
change timing budgets or infer a performance repair from aggregate logs.

New first vehicleStart plus continued normal supply permits a larger acceptance
window: single-r26, same control source38a562ee/original DLL/standard6laps600simsec,
660hostsec limit to observe simulator timeout and teardown. Preserve launch/actual
publication failure stop conditions and all controller budgets. This differs from
repeating a terminally failed120sec run. Temporary interval observer is retained
for this newly reached course range; revise its original120sec retirement boundary
explicitly here, then review deletion after the full-course attempt. It remains
observation-only and cannot affect authority. Saved r24 failures, physical/Recovery/
prior r80D2 timing and M4–M6 remain open. [Evidence](current-wall-interval-live-evidence.json).
