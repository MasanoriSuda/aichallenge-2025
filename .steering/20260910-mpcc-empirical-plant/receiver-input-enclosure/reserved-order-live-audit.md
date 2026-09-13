# Reserved-order live validation and remaining source boundary

2026-09-13. Runtime7468cf46, build135/package122, standard single-r20with originalDLL.
120host seconds:4257callbacks,max17.981554ms,534normal sends,last1142/job1141/source692.
1111refused before publication after its original deadline; no actual late send.
One selected authenticated Rejoin970/source544, no Rejoin completion, vehicleStart
or laps. Game-wideStart is present. Public maxspeed1.073466063mps, clock200Hz,
velocity/steering28.571Hz,IMU20Hz,pose49.99Hz,commands36.11Hz. UserJSON/DLL restored,
source/binary hashes unchanged. Different world fromr19; no causal count comparison.

13reservation-invalid records comprise9after-deadline and4before-deadline refusals.
Only1of the9late records is reserved (1054/job1050,58.200837ms);8have zero priors.
577/780are timely but captured source generation is inactive; prefix is consistent.
899/1144report actual-prefix mismatch but have no dedicated complete capture. Keep
these causes separate; do not bypass generation, ledger or expiry checks.1054has
9.320070ms domain cost and one-positive-packet final programme. The local order fix
preserves safety and improves the frozen1223case; it does not close every live job.

Source supply eventually sticks at Store704/accepted454 while workers still run.
The first observed missing-Stop-geometry trace is1242at20.009999552. Its QP accepts
without a wall profile and the final physical owner rejects all three missing wall
arrays. In mpc_controller_cpp's current-envelope producer, acceptance also requires
preferred_lateral_contained, but the rejection log reports only valid/feasible and
scalar search bounds, not the selected interval. No exact later producer input is
captured. Actual hard-clearance contact remains possible; no geometry or safety
relaxation follows from that incomplete diagnostic.

Earlier complete post-motion Rejoin sources532/548/649are available: wall-refinement
QP failure, outer exact wallstage719, and initial QP failure.548is a new independent
world with nonzero semantic initial speed0.096260268mps. The fixed preparation
library can be compared here before changing source generation; zero acceleration
must not be described as stationary motion at this initial speed. Missing geometry,
physical observations, current/retained bounds, r80D2actual-send and M4–M6 remain.
[Sealed evidence](reserved-order-live-evidence.json).

R747fixed four preparations on548yield3raw physical positives; longer left rejects
wall318. The cold original-controls raw conversion rejects progress regression
before wall, so it is not a replay of the old live719iterate. R748normal cold source
re-solve reproduces wall720(class retained); right-drive2initial tangent lets all
QP controls/objective remain free and passes full source state/wall/dynamic/Stop
in81.08ms; right-drive4rejects wall957. Paired537also accepted right-drive2(R724).
[Holdout evidence](rejoin-preparation-holdout-evidence.json). Numerical initialization
remains diagnostic; declare bounded producer/serialization contracts before code.
