# Evidence

## Static identity

- Host source tests: `95 passed`.
- Docker installed-space tests: `95 passed`.
- raw TinyLidarNet source/install SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`.
- spatial adapter source/install SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`.
- Launch defaults log the frozen `fixed_lidar_brake`, `0.8 m/s2`, `4.6 m/s`,
  spatial-authority-enabled and recurrent-disabled configuration.

The first host test invocation lacked the source package on `PYTHONPATH` and
stopped during collection.  Re-running the exact tests with an explicit source
package path passed, as did the independently installed Docker package.  This
is an environment/overlay observation, not a waived test failure.

## Packaged-default single vehicle: pass

Run: `output/20260902-e2e-submission-freeze-single`

- Laps: `84.5182 / 83.9035 / 83.8086 s`; total `252.2303 s`.
- Finish: 3/3; penalties: zero.
- Distance: `1017.046 m`; mean/max speed: `3.8004 / 4.4570 m/s`.
- Longest low-speed and positive-acceleration stall: both `0.0 s`.
- Runtime recurrent checkpoint/path/process: absent.
- Motion and strict competition reports: pass.

This matches the accepted external process-shadow baseline (`252.2603 s`)
without running any recurrent diagnostic work.

## Packaged-default three vehicle: reject

The first launch attempt,
`output/20260902-e2e-submission-freeze-peer`, is invalid model evidence.  AWSIM
remained in `Spawned` after all three EKF triggers and never entered `Start`;
no production command episode was evaluated.  Containers were fully removed
and the unchanged configuration was started once more.

The valid failure run is
`output/20260902-e2e-submission-freeze-peer-v2`, evaluated on d3.  It started
normally but entered a sustained physical stall:

- distance before termination: `374.426 m`;
- first sustained stall: `105.90 s` after bag start;
- longest low-speed interval: `54.914 s`;
- positive-acceleration stall: `32.073 s`, `917` samples;
- at onset: speed below `0.15 m/s`, command `+0.8 m/s2`, frontal clearance
  `3.278 m`, closest/left clearance `3.201 m`;
- after the trapped pose moved the frontal return below `3.0 m`, the existing
  longitudinal owner correctly changed to `slow-clearance` and `0.0 m/s2`;
- scan freshness and spatial inference remained valid, so this is not a model
  loading, stale-LiDAR or inference exception.

The offline interaction comparison used the exact packaged spatial artifact
and the prior accepted peer bag.  In the ten seconds before the new stall,
there were 23 coherent `side-clearance` samples.  The diagnostic executed-
teacher policy requested a mean `+0.1510 rad` residual while the packaged model
predicted `-0.0491 rad`; teacher residual MAE was `0.3577 rad`, projection
deficit occurred in `100%` of those samples and published steering remained
toward the detected obstacle in `30.43%`.  The accepted reference had no stall.

This classifies the blocker upstream of the later zero-acceleration plateau:
the static spatial policy chooses the wrong lateral response during a coherent
peer-side hazard, physically traps the kart, and only then reaches the
longitudinal slow-clearance state.

## Decision

Reject this tree as a multi-vehicle submission freeze candidate.  Retain it as
the single-vehicle packaged baseline; do not change its production defaults or
create a submission archive from a failed peer Gate.

The next bounded Slice must test an already-existing, outcome-qualified
speed-committed teacher in the same peer world.  It may create training labels
only if the teacher itself finishes with zero penalties and stall.  The failed
student command is not a teacher label, and obstacle thresholds, steering
bounds and retry counts may not be tuned to hide the failure.
