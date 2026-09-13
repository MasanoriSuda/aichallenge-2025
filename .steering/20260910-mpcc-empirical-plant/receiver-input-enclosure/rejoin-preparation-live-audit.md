# Rejoin preparation live boundary

2026-09-13. Runtime6f42d134, build136/package123, standard single-r21, originalDLL,
120host seconds.4248callbacks,max19.529216ms,408normal sends,last912/job909/source543.
No pre-send deadline refusal or actual post-send violation; public max1.151937842mps.
Clock200Hz,velocity/steering28.571Hz,IMU20Hz,pose50Hz,command35.97Hz. GlobalgameStart
is present; vehicle states spawned/grounded/ready, no laps. Source/binary hashes
unchanged, runtime stopped and protectedJSON/DLL preserved.

The new Rejoin preparation is never eligible: no Rejoin intent/source trace or
actual Rejoin send. Recovery first suspects at813 while normal remains allowed,
then913holds stop.914's two-prior reserved job completes before its original deadline
(42.322972ms including21.087507ms domain), but its source generation is inactive
and actual prefix differs after the override. Seven reservation-invalid records
comprise4late zero-prior and3timely reserved cases.786is also timely/inactive;
583has no dedicated complete capture. Do not attribute every invalidation to cost.

Recovery then fails direction selection and reaches SafeStop. Logs report no current
contacts, complete V2X and a rejected course candidate, but omit the full collection
of attempted primitives, individual hard-wall/course outcomes and immutable inputs.
Store553remains certified while new source supply is suppressed by the supervisor.
This is separate from source537/548offline preparation proof and from r20's later
missing wall profile. No physical infeasibility or safety waiver follows.

Detector speed comes from odometry; the public speed peak at10.115sec and incident
elapsed2.50sec are not the detector's observation at913. Its configured stopped
confirmation is0.25sec, moving release0.35mps;913trace reports0.31mps. These alone
do not establish a false detection. Preserve the thresholds and inspect exact
direction candidate inputs first. Next bounded observation/replay at the producer,
then supported structural repair and remaining full race/M4–M6 validation.
[Sealed evidence](rejoin-preparation-live-evidence.json).
