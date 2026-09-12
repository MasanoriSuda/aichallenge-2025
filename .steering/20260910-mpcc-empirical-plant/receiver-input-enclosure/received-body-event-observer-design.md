# Coupled received-body event observer: next offline comparison

2026-09-13 JST. No production change yet. Baseline/rollbackb6c6d04f; controller
fe976b39/build121. R568supports interpolation where exact causal upper endpoints
exist, but it cannot repair unsupported held channels. R569rejects blanket linear
extrapolation after lateral/tire regressions. Do not repeat these unchanged arms.

Proposed producer repair: initialize u/vy/body-r/physical-tire at the latest exact
common source epoch present in all3immutable received queues and covered by the
actual published-input history, including both channel delays. Velocity/tire35ms
andIMU50ms periods suggest common epochs, but support must be proved from actual
integer source stamps, not assumed or approximated with tolerance. No manufactured
zero initial body. Missing common epoch/history is explicitly unavailable.

From that common epoch, integrate the unchanged native body/tire model, with only
actual already-published command history and existing nominal delay semantics.
Insert each received velocity,IMU and physical-tire measurement at its real source
epoch, simultaneously for equal epochs; stop at the pose epoch. Never consume a
source newer than pose in this arm, or a sample outside its captured receipt.
Anchor global x/y/yaw at the actual pose; body dynamics must be verified invariant
to the arbitrary initial global position/yaw used for a body-only reconstruction.
Keep raw observations/receipt/endpoints/model/event list and the derived pose-epoch
state separate. Post-publication desired input remains governed by actual history.

First perform bounded offline comparison using the sealedforce-r3/R7captures with
exact captured command support. Preserve unavailable cases rather than borrowing
later history. Compare originalheld/frozenbracket and the coupled model on the
same16physical matches where possible, including source883/1005and rest/launch.
Record all component errors and support failures. R3is inspected evidence, not
an untouched holdout. Do not add control margins, timeout windows, retries, clamps,
new solver budgets, affinity or independent normal authority.

Before promotion: prove deterministic event/clock/reset semantics, validate raw
versus derived state contracts through every model/adapter/tube/current observer
consumer, reseal all affected identities, preserve original physical/clock
negative replays, build/package tests and fresh closed-loop acceptance. A point
estimate and lower average error alone do not establish physical uncertainty bounds
or full acceptance. Keep current production unchanged while comparing this arm;
any integration must retire the old mixed-epoch producer in the same slice.
