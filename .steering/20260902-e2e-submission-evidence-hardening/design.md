# Design

## Competition provenance

Extend the existing launch-log provenance parser instead of introducing a
second submission checker.  Parse the following additional launch values:

- spatial checkpoint path and expected SHA-256;
- spatial authority enabled and maximum correction;
- recurrent checkpoint path and authority enabled.

The analyzer independently hashes the raw and spatial files supplied on the
command line.  Runtime path/SHA/authority expectations are optional for
general historical analysis, but the video checklist supplies every
production expectation and therefore fails closed.

## Readiness classification

Keep motion admission as a required liveness signal, but add the mixed-peer
competition report as a separate mandatory Gate.  A multi-vehicle candidate
requires both reports to pass.  This prevents a moving but unfinished or
penalized peer run from being promoted.

## Documentation

Describe the spatial adapter as continuously evaluated on admitted scans,
with the final command saturated to `+-0.64 rad`.  Name the mixed-peer setup as
two MPC peers plus one E2E student, and describe seeds as training-unused start
seeds rather than an unrestricted unseen environment.

## Compatibility

ROS topics, services, launch defaults, checkpoints and runtime control are
unchanged.  Only offline evidence-tool arguments and documentation change.
