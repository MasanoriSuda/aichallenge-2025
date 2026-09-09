# Causal force-step model audit and implementation

Continue the user-authorized M1–M6 completion plan from094941b5; participant1c4f377e.
No separate approval at audit/implementation boundaries. Full acceptance remains open.
Use the current root/package authority, architecture and evidence rules.

Earliest unresolved boundary: wire input and state-to-state physical response.
Previous conditional fit uses future measured state and lacks contact/force epoch;
flat response across rest/input changes is rejected. Do not tune gains or safety bounds.

Bounded next observation: generated-only CIL probes at Vehicle.FixedUpdate entry/end
and the actual Wheel.UpdateWheelForce values, alongside the existing4input probes.
Capture Rigidbody physics position/rotation/velocity/angularVelocity (not interpolated
Transform), actual fixed step, pre/post accumulated force/torque, sleep, each grounded
wheel's actual lateral force and drive acceleration. Preserve original branches/data.
At most first single lap and first dev2moving override+teardown; no performance claim.

Compare step response using measured actual force versus components without fitting
future state. Then compare causal candidate rollouts using only current ROS observations
and published input history. Separate private diagnostic oracle from deployable inputs.
Selection must explain rest, acceleration, braking, steering/contact and application
uncertainty, with native failures and identity/schema contract before production edits.

Model change must own prefix/QP/tangent/physical/Stop/artifact/async/serialization in one
slice. Retire replaced integrators and authority paths. No old-schema authority reuse.
Same-world A/B/C/D comparisons use one model/hard constraints; all-failure stays Unknown.
Follow focused/native/package/build/replay, single/dev2, all intents/multicar/gates,
same-artifact submission/eval, documentation/registry/local commits. Rollback094941b5.

Initial acceptance for diagnostic oracle: static original CIL equivalence after removing
7probe calls; no probe errors; raw fixed-step state/force records finite with explicit
gaps; actual wheel force sum agrees with body accumulated force delta (float rounding
reported). Missing contact/engine forces remain residual, not tuned away. No use of
current/next raw measurements as a hidden future decision input.
