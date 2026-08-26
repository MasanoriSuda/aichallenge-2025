# Task List

- [x] Reproduce the failure in a V2X-empty `make dev` run.
- [x] Separate the first wall event from downstream QP and Recovery symptoms.
- [x] Decode desired/measured steering around the first wall event.
- [x] Select or reject H1--H3 with recorded evidence.
- [x] Add failure-first contract tests for physical/desired/wire separation.
- [x] Implement the minimum structural correction.
- [x] Run focused tests and the full package test suite.
- [x] Run `make autoware-build`.
- [x] Reject the stale 1.5 inverse with a third delayed-static plant fit.
- [x] Validate the evidence-backed 1.435 AWSIM actuator calibration.
- [x] Identify the separate report-to-yaw response from four bags.
- [x] Add the response-steering state and bind all normal producers.
- [x] Version the formulation and artifact schemas as seven-state v2.
- [x] Pass all 46 package test targets.
- [x] Reject the dual physical/desired integration-origin contract with
  dynamic evidence from `20260826-202338` and `20260826-203435`.
- [x] Add a failure-first contract for command-state versus response-state
  ownership.
- [x] Make the last serialized command the sole fresh/retained command
  integration and reachability origin.
- [x] Keep measured/yaw-derived steering as response observation only.
- [x] Re-run all 46 targets (zero failures).
- [x] Repeat an initial clean Track/Cruise Gate (`20260827-010414`): the
  command/wire serialization contract passed, exposing a separate retained
  suffix-authority defect.
- [x] Update `audit.md`, validation plan, and canonical specification.
- [ ] Complete the six-lap Track/Cruise acceptance after resolving the
  separately observed physical speed-collapse near WP72--76.
- [ ] Commit the accepted change without generated result JSON files.
