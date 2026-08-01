# Design

## Rejoin regression

Extend `RejoinAlignmentProgressTracker` with a lateral-capture latch. Once absolute lateral error
has entered `max_lateral_error_m`, report regression when it leaves that envelope by more than
`lateral_regression_margin_m`. `RecoverySupervisor` stops and returns to its existing bounded
clearance reassessment path immediately instead of waiting for the no-progress timeout.

The checked-in race configuration uses a 0.20 m margin. With the existing 0.50 m completion
envelope this interrupts a rejoin at 0.70 m absolute lateral error after capture, well before the
multi-metre overshoot observed in the run log.

## Snapshot-gated aggressive retry

Record the recovery snapshot whenever an aggressive retry is issued. The snapshot contains the
candidate direction, gear/freshness, step mode, contact count, rear static/V2X completeness and
clearance, rejoin clearance, collision/course-worsening flags, and path-relative lateral/heading
errors.

When the retry returns to `SafeStop`, compare the current snapshot with the last retried snapshot.
An unchanged snapshot remains stopped and monitored. A discrete state change, contact-count
change, or configured path-relative pose change re-arms one retry. A new recovery episode clears
the remembered snapshot.

## Direction policy

`forward_fallback_unlocked` removes a Reverse-only constraint but does not itself request permanent
Forward ordering. Forward is preferred only for current measured Reverse course worsening or an
active course-directed Forward escape. Candidate selection therefore falls back to Reverse when a
previously unlocked Forward rollout is no longer executable.

## Compatibility

- Pure-core structs are internal C++ interfaces in the participant package.
- ROS graph and evaluation contracts are unchanged.
- New YAML keys have safe defaults in `SupervisorConfig` and are explicitly set in `config.yaml`.
