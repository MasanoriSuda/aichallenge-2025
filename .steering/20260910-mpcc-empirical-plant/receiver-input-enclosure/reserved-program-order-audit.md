# Reserved programme ordering and live source validation

2026-09-13. Authorized autonomous continuation, local commits and no push.
Baseline40d1664b; source slice validated by build134/package121. All M1–M6 work
continues; race/submission acceptance is incomplete.

single-r19 ran120host seconds with original DLL and fixed source/binaries.
R732:4463callbacks,max22.085974ms,0over25;2325normal sends,last4435/job4430/source5002.
Pre-send rejection863 and1194are retained; no actual post-send deadline violation.
At least5authenticated Rejoin sends and2Rejoin completions. Runtime ends while
normal sources are still supplied; repeated stopping and Recovery remain.
R733clock200Hz,velocity/steering28.571Hz,IMU20Hz,pose50Hz,command38.03Hz.
Public maxspeed1.433313489mps. Game-wide Start IS present in Unity; vehicle states
remain spawned/grounded/ready and no laps. Prior unqualified Start wording used
vehicle observation only and must not be read as a global handshake failure.
UserJSON/originalDLL restored; source/binary hashes unchanged. This world differs
from r17/r18; no deterministic attribution to source-state code is claimed.

## Earliest observed dependency and reproduction

The post-Rejoin recorder finally receives actual eligible input. Its1193Rejoin
command, original certificate706/currentdecision1193 and actual transaction match
normal log. First expired reservation is1223/job1219/source719 with2prior packets:
planning19.194999570,nominal19.264999573,deadline19.289999573,current19.294999568.
Only one source attempt ran:69.374142ms plus19.114160ms starting domain; wholeworker
88.498982ms. This is not a publisher deadline extension or an actual late send.
R734both attempt and parent reconstruct identical programme and domain fingerprints.
Compute~64.7ms plus18.84ms domain reproduces the cost without live adoption.

R735per-candidate timing showed2attempts, but its inner scheduled reason fields are
unset defaults and cannot diagnose the rejection. R736observes the real scheduled
proof owner: first full-horizon programme spends~40.72ms and rejects an applied wall
bound at21.084999570 (Reason::WallRejected, prediction ValidationRejected). Nominal
440dense samples/rest2.2s/max1.206426653mps pass; semantic ceiling11.111mps is not the
cause. The reservation-span candidate (3positive publisher packets, then braking)
passes in~24.38ms; nominal30samples/rest0.15s/max0.181721355mps. Original full physical
wall proof must remain. Earlier source-horizon velocity-failure hypothesis is false
for this exact world.

## Bounded downstream comparison and remaining producer work

R737predeclared4orders reuse ONE solved nine-state source and all original semantic,
world, pending inputs, clock, model and hard proofs. Existing order costs64.53+18.84ms.
Reservation-span first costs24.54+18.83ms and reproduces the SAME full certificate,
programme and domain fingerprints with one candidate proof. Bootstrap first and
normal-path first pass in~30.4/30.8ms including domain, but choose shorter different
programmes. Timing alone grants no normal authority. This is a downstream programme
study, not a replacement for package Mission/SQP A/B/C/D architecture comparison.

The candidate producer tries a long programme before the span already determined by
its reserved publisher dependencies, although the latter is the final accepted
programme. The rejected long proof consumes the arrival budget. Domain cost is a
contributor; the final clock rejection correctly protects publication. Proposed
next slice: establish holdout/negative behaviour, then prioritize the existing
reserved span, preserving the full-horizon and bootstrap alternatives without
repeating any member or increasing the finite population. Zero-prior bootstrap,
full solved Stop and rear-peer safety must retain complete proof. No timeout,
parameter, clock, physical/model or safety relaxation. A rejected longer tube does
not establish physical infeasibility of the scene.

Current/retained velocity semantics, source preparation, physical observations,
r80D2actual send, vehicleStart/laps and full M4–M6 remain separate obligations.
[Sealed evidence](reserved-program-order-evidence.json).
