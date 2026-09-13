# Recovery direction live input and native replay

2026-09-13. Runtime1b478b27, build137/package124, original-DLL single-r22/120hostsec.
4329callbacks,max24.695564ms,1133normal sends,last1781/job1776/source2067.1489is a
pre-send deadline refusal, no actual late sends. Selected trace confirms at least
three actual Rejoins945/495,1178/875,1222/957 and one Rejoin completion. Preparation
sources775/1079/1163certify, but their mapping to those particular sends is not
captured. GameStartpresent; vehicleStart/laps absent. Public max1.741180062mps,
clock200Hz,velocity/steering28.571Hz,IMU20Hz,pose50Hz,commands36.758Hz.
Source/binary/protectedJSON/DLL hashes retained and runtime stopped.

The first failed CheckClearance direction is1921at37.139999169, callback1.668619ms.
Its map/pose/footprint and all4actually evaluated trials are complete. NativeR764
replays exact cells, result reasons, contact/pose counts, endpoints and course
previews. Current footprint is clear, V2X summary complete/clear, reverse-onlytrue
and forward-probefalse. The0.8mrejoin lookahead is physically clear but worsens
lateral error. All three8mreverse candidates collide: Straightat6m, Left3.55m,
Right7m. Their endpoint course previews also reject, but physical rejection occurs
first. This corrects the possible interpretation that course guard alone caused
the unknown direction. Neither infeasibility nor a safety waiver follows.

Earlier normal travel clears the2madaptive reset threshold. A later reverse episode
selects4m, starts Reverse, then reports rear_static_blocked/rear_hazard_appeared and
escape_not_confirmed. The aggressive retry increases target to8m before1921. The
new record proves this later candidate failure, not the earlier moving hazard's
producer. Next compare configured base4m and a bounded piecewise8m candidate with
the same map/footprint/course/steering constraints. A shorter target is a changed
request and cannot satisfy the original8mgoal by declaration. A raw kinematic
witness is also not a nine-state certified normal trajectory or actual execution.
All physical-observation/current-velocity/r80timing and M4–M6 obligations remain.
[Sealed evidence](recovery-direction-live-evidence.json).
