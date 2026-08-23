# Requirements: Overtake retained live authority gate

## Objective

Separate counterfactual rosbag-replay rejections from defects that occur when
the vehicle actually responds to the current controller. Use the already
committed typed fresh/retained shadow telemetry in an AWSIM `dev2` run before
changing production authority or continuity policy.

## In scope

- Run the accepted `46c853b` controller without source/config changes.
- Record fresh/retained coverage, rejection distribution, physical rejects,
  solver failures, callback timing, authority traces, and lap outcome.
- Correlate the first uncovered eligible interval with phase, target, progress,
  course-frame, and solver state.
- Decide the next implementation Slice from the earliest live invariant break.

## Out of scope

- Production Overtake authority promotion.
- Timeout, horizon, progress tolerance, wall margin, solver, or cost tuning.
- New fallback/lease/flag.
- Treating replay-only progress divergence as a live root cause.

## Acceptance

The run is useful only if Domain 1 reaches Overtake and emits typed outcome
telemetry. A source/config change is prohibited until the first live uncovered
canonical cycle has an evidence-backed upstream cause.
