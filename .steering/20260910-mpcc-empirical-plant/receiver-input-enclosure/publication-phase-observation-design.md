# Callback phase observation, baseline 8d85234b

Continue authorized M1–M6. Dev2-r50 is failed. Across complete logs, D1/D2
have10/7pre-publication rejects, all late and none before the original opening.
There is no post-send violation. First moving D1dispatch874at0.17m/s has
nominal9.639999803, callback9.644999784, before9.669999783, deadline9.664999803.
Its wall callback is19.308064ms; current admission succeeds with an independent
physical proof, and the later final guard correctly refuses the late packet.
The alarm repairs the early-wake producer; it is not whole deadline acceptance.

Detection gap: per-phase detail is emitted only above25ms wall time, so it
omits this19ms callback that crosses an earlier original ROS deadline. The
main current proof and its surrounding source/problem/Recovery work are not
separated on the same rejected decision. R49publisher intervals do not establish
R50callback root cause. Existing CPU affinity and grace experiments remain
unpromoted; no parameter tuning or authority relaxation follows from elapsed time.

Add observation-only current-request/current-proof wall and thread CPU time to
the existing admission line. Emit existing callback phase fields also when the
publisher returns failure, with an explicit failure label instead of claiming a
wall overrun. Preserve entry/MPC/Recovery ROS clock samples to distinguish the
clock's advancement from steady CPU/work time. Keep all guards, source/context,
worker scheduling, alarm, physics, rates, timeout and output order unchanged.
New clocks are observation overhead; POSIX thread CPU failures are explicit NaN.

Validate source contracts, build/package and exact R50current proof/pre-send
rejection replay. Then committed dev2-r51captures the first exact phase failure.
Use that measured producer boundary before changing callback architecture or
cost. All M4–M6, Stop/rest/restart, all intents/Mission/sibling/Store/Recovery/
async, repeated races/gates and same submission/eval remain open. Rollback8d85234b.

R305 reproduces the exact original input fingerprint and independent current
physical proof (6.832035ms in isolation); the saved late
pre-send clock remains rejected, and an original in-window clock is accepted.
No current packet was sent or added to this diagnostic ledger. Buildr91passes
26packages; testsr79passes2545records/65groups, zero errors/failures/skips;
source105pass. New committed diagnosticdev2-r51is next.
