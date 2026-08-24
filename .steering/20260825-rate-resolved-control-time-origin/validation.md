# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline commit: `6d8ce09 refactor(mpcc): bind rate-resolved shadow causally`
- Simulation run: `output/20260825-074847`
- Domains: d1 and d2
- Authority: six-state candidate remained `authority=shadow, selected=0`
- User-owned `aichallenge/result-summary.json` was not used as evidence and is
  excluded from this Slice.

## Failure-first evidence

Before the producer fix, focused tests demonstrated that:

- an artifact used callback capture time instead of the latency-predicted
  control origin;
- artifact completion was incorrectly required to occur after a future control
  origin;
- a moving obstacle crossing only during the 0.05 s latency prefix was
  accepted because the peer was frozen at elapsed time zero;
- a control origin inconsistent with the timed prefix was accepted.

After the fix, the new tests reject the two invalid retained cases and bind the
artifact to the explicit future control origin.

## Static verification

- Focused six-state shadow tests: 7/7 passed.
- Focused retained current-world tests: 16/16 passed.
- Full `multi_purpose_mpc_ros` package CTest: 49/49 passed.
- `make autoware-build`: passed, 25 packages completed. Only the existing
  setuptools deprecation warning was emitted.
- `git diff --check`: passed.

## Dynamic verification

The valid `make dev2` run is `output/20260825-074847`. The earlier
`output/20260825-074524` startup did not establish an AWSIM session and is not
combined with this causal timeline.

Both domains emitted a non-zero control delay of exactly `0.130000 s` once the
shadow plan became available. Zero-valued timing entries were only the
intentional no-plan startup telemetry.

Representative d2 terminal window:

- six-state solve: 81 submitted, 81 solved;
- physical wall proof: accepted;
- retained current-world proof: 81 attempted, 81 accepted;
- command candidates: 81 attempted, 81 available;
- observation/control origin: `208.444995 / 208.574995 s`;
- prediction delay: `0.130000 s`;
- retained cursor elapsed time: `0.020000 s`;
- dynamic samples: 491;
- all commands remained shadow-only.

The 40 Hz callback telemetry contained one isolated overrun in d2:
`26.559 ms` maximum against a `25.000 ms` period. All other reported windows
had `overruns=0`; the final d2 shadow solve maximum was `2.281 ms` and retained
proof maximum was `0.260 ms`. This is not evidence of a sustained timing
regression, but it remains an acceptance item for production promotion.

Startup missing-odometry fail-safe messages and shutdown RViz/orchestrator
messages were outside the active control interval. No timing-provenance reject,
invalid artifact, or production authority selection was observed.

## Invariant result

The controller now proves one continuous dynamic timeline:

1. observation pose to latency-predicted control pose over `0 .. delay`;
2. same-time connector at `delay`;
3. retained suffix beginning at `delay`.

The repaired producer is the explicit control-effective timestamp and timed
ego prefix. No threshold, lease, fallback, or new normal authority was added.

## Remaining gate

This Slice does not promote the six-state command. The next vertical Slice must
atomically connect six-state Track/Cruise fresh/retained commands to production
and delete the five-state Track/Cruise normal owner. The isolated callback
overrun must be included in that Slice's multi-lap timing acceptance.
