# Shared body and actuator model

Continue M1–M6 from9bca3af6. No new approval or completed-task claim.
Current MPCC remains1c4f377e; measured heading is now repaired. Full normal and
Stop model still interpret wire acceleration as net velocity derivative and use
a kinematic yaw surrogate. Direct force/state comparisons contradict both.

First complete deployable-input comparison. Freeze wheel/contact parameters from
single16..35s training only. Initial inputs come from source-stamped public ROS
VelocityReport, transformed IMU and SteeringReport; no private body/contact state.
Use only received, nonfuture sensor samples at each anchor. Report sensor skew.
Predict relative displacement from zero origin; private Rigidbody is scoring only,
not an absolute localization initializer. Compare old law versus dynamic body,
with the same declared command schedule and explicit actuator boundary.

One arm holds the last already-published command throughout the future. A second
uses a prescribed future publication schedule as exogenous test input (analogous
to a candidate control sequence), never future sensor/contact/actuator values.
Both retain original source timestamps. Published source time is not actual DDS
receipt/application. Do not tune a delay to reduce the failure score. Report the
remaining application uncertainty explicitly and do not claim an observed maximum
is a guarantee. This test is necessary before promoting any body model.

Implement one shared kernel, source-state/command history, rest/gear contract,
model fingerprint/schema and all consumers only after this comparison supports
selection. Canonical QP, nonlinear sweep, delay prefix, Stop-to-rest, artifact
cursor and async validation must all use the same body/input meaning; retire
replaced integrators/overrides. No old-schema authority. Existing nine-normal and
four-Stop architecture comparisons on928/new933 are Unknown; new-model comparison
must reseal all arms with the same model/world/hard constraints. No gain/limit/
solver/margin/grace tuning. Follow M3–M6 and commit only actual accepted work.
