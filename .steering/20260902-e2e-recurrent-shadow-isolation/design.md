# Design

## Root cause

The rejected peer512 Gate measured an authority-disabled recurrent model in
the same ROS scan callback that owns the published command.  Under three-peer
load, the diagnostic added 30.91 ms mean and 151.06 ms maximum callback work.
The callback also shares wheel-speed freshness with production spatial
authority.  Diagnostic load can therefore perturb the baseline that it is
supposed to observe.

Changing the 0.1 s speed timeout or recurrent coverage thresholds would hide
that ownership defect.  The correction is to remove non-authoritative work
from the command critical path.

## Runtime structure

The core builds an immutable recurrent input from the already-computed Conv5
features and the admitted production-spatial baseline.  With recurrent
authority disabled, the ROS node publishes the production command first and
submits that input to a one-worker latest-wins executor.

The executor owns recurrent hidden state.  It can have at most one running
item and one pending item.  A newer submission replaces the pending item and
increments `dropped`; it never accumulates a FIFO backlog.  Completed results
are returned to the ROS thread only for telemetry.

The worker result cannot alter steering, acceleration, watchdog state or
spatial-authority state.  If recurrent authority is enabled, async isolation
is disabled and the existing synchronous evaluation remains the only command
path.

## Lifecycle

- Missing production-spatial input skips that diagnostic sample without
  resetting hidden state.
- Queue replacement skips an obsolete diagnostic sample without resetting
  hidden state.
- An inference exception resets the worker-owned hidden state and is reported.
- The LiDAR watchdog and node shutdown explicitly reset or close the worker.
- A result older than the command sensor timeout is reported stale and is not
  counted as admitted evidence.

## Verification

First run package unit tests and launch/interface contract tests.  Dynamic
verification then compares production-only and recurrent-shadow runs:

1. single vehicle, exact checkpoint identity;
2. deterministic three-vehicle MPC-peer load;
3. unchanged recurrent authority count of zero;
4. production callback scan rate and tail latency;
5. explicit async submitted/completed/dropped/stale/error counts.

The later bounded-authority A/B remains rejected until both dynamic shadow
Gates pass.
