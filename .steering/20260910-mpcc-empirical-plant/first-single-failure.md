# First nine-state runtime failure and map-support repair

Run `output/20260910-nine-state-single-r1`, Domain1, commit226f93e0.
First moving Emergency: decision2804, observation56.729998731,
control origin56.859998731, actual9.76m/s. Failure snapshot
`000000002804-d44d2095147eb694-cruise-side-neutral-physical-proof-normal-authority-unavailable`.
The observed failure world fingerprint is15297919333913507476.
One lap was logged42.495s; this run is rejected, not a completed six-lap trial.
84callback windows: maximum20.035ms, reported overruns0. No missing causal body
input failsafe. Startup/shutdown missing-odometry events are separate.

## Reproduction

`replay_first_failure.py` and `replay_revalidation.cpp` read the original nine-state
artifact and actual current/previous Requests with zero solves. All recorded model,
velocity, yaw rate, physical tire, source/publication epochs and grids are retained.
`output/20260910-nine-state-revalidation-r1`: decision2803 accepted;
2804 terminal-contingency-unavailable, independent Stop physically rejected.

Diagnostic-only adapter in `output/20260910-nine-state-stop-diagnostic-r1`:
the source frame is[40.756825379741208,60.734594048870434]m. At rejection,
progress60.733882992328894 would advance to60.740300970598469 while still moving
u1.283203988759841m/s. No wall or dynamic proof was reached. The alternative
path-reference attempt similarly exceeds the frame withu0.75814841138162148m/s.

Root cause: the physical coordinate and Stop geometry producer truncated its
map support to the normal QP progress domain. The empirical wire/body law needs
more stopping support than the old wire=net assumption. A retained current origin
moves forward while the source map window stays fixed. At2804 remaining map
distance is19.0338463966m. It is a data-coverage failure, not physical infeasibility.
Full normal continuation also fails its approximate lateral box; the existing
publisher-prefix plus certified complete Stop is valid only if Stop can be built.

## Fixed-condition comparisons

Default A/B/C/D tool on the same failure world:
`/tmp/mpcc-nine-state-first-failure-comparison-r1.log`. Fresh persistent A solves
and certifies. B/C/D avoidance arms require an active peer and are not applicable
to this empty-peer Cruise scene; their candidate rejection is not a feasibility
result. No solver budget, hard constraint or production control changed.

`output/20260910-nine-state-stop-support-r1`: reconstruct the original map producer
from the savedDomain1CSV; all21 original knots match within1e-8m. Add30actual
successor map knots only, keep solved controls/current Request/wall/peer limits,
and reseal diagnostic candidate18227583145526969959→382167073820959060.
The same2804 now accepts publisher-prefix plus complete Stop and independent
Stop; swept wall and dynamic proofs pass. This is a diagnostic, not publication.
The appended approximate width hints reuse the final width for this comparison;
the production producer reads actual map widths for every appended waypoint.

## Producer repair and acceptance

Allocate actual course data beyond the normal QP domain using a straight
shared-model nominal Stop rollout at the current speed limit, with one publisher
interval at maximum wire input followed by braking. This is a map-allocation
hint, not a universal bound. Existing finite map/extension limits still apply;
actual curved Stop must reach rest within available support and pass every proof.
No QP horizon, progress box, wire limit, physical tolerance or solver setting changes.

Seal extended Stop curvature/width support separately in the immutable source,
serialize/hash it, copy it through physical binding and comparison, and rebase its
local progress when a candidate origin moves. Keep old saved snapshots readable.
Regression checks cover wire/body stopping support, serialized support identity
and missing coordinate coverage. Build/package and fresh single must pass after
the repair, followed by the remaining M3-M6 work. Rollback is a focused revert of
the upcoming map-support commit; do not alter226f93e0 or protected user results.
