# Stop reference feasibility

User-authorized MPCC completion. Preserve one canonical normal command and
the exact current-world Stop certificate. No change to speeds, gains, margins,
solver limits, publication clock or ROS contracts.

Confirmed boundary: single observation run, decision9401/sequence8769,
control219.729995s/speed8.131m/s. Existing zero-lateral Stop is rejected, while
the artifact-local normal-path profile passes the same retained evaluator,
including current-world join, nonlinear rollout, wall and dynamic checks.
Comparison takes0.446ms and supplies no command. Aggregate normal authority
is lost later at9407/sequence8770; these two sequences must not be conflated.

Frozen11046 also accepts the normal-path profile and a bounded lateral target
scan, but rejects zero target. A longer free-control Stop solves numerically
and hits wall. The B/C/D overtake generators are inapplicable without a peer.
This identifies the hardcoded lateral target as an incomplete Stop feasibility
search, not a physical inability to stop or a need to weaken acceptance.

The correction is to evaluate the solved-path reference as a Stop feasibility
candidate. Each selected reference still needs the complete existing proof.
Keep the existing track-reference hypothesis for sources whose short/zero
progress path does not cover braking or whose profile fails physical proof.
That reference must independently pass the same proof. No plan/clock is held
past failure, no failed candidate is executable, and no profile is extrapolated.

Rollback to the preserved pre-promotion controller:
output/20260908-mpcc-normal-path-stop-observation/binaries/mpc_controller_cpp
(0c9c095fb75d8d4be3091ee6ed4cde29271464b3d34d5edc65c858d49c930213).
