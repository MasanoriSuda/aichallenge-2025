# Float32 command reachability before certification

2026-09-12 JST. Baseline/rollback 3ac41a0e. M4–M6 remain incomplete.

Standard r58 stalled before normal control: initial GNSS/odometry recovered,
then clock stopped. A six-second read-only Domain 1 subscriber discovered
awsim_d1 publishers but received no clock/GNSS/state. Main Unity thread was
waiting on a futex; this is not a root-cause stack. Clean same-input r59 reached
normal control. Preserve r58 as startup-inconclusive, not controller acceptance.

R59 first D2 pre948 is within its original window: previous actual12.364999723,
nominal/before12.389999723, deadline12.414999723. Full source and current-domain
proofs pass. The final wire step is0.027671965157113423rad, while the unchanged
rate*age+tolerance bound is0.027671960261689516rad. The double-valued reachable
endpoint was serialized first to float32 physical steering, then calibrated,
then serialized again to float32. Its outward rounding exceeds the original
bound by4.895423907e-9rad. The candidate producer selected a real-valued endpoint
without enforcing reachability of the actual representable command. The final
guard correctly rejects. R347 reproduces source/current identity and this guard.

R59 D1 pre1117 instead misses its original deadline after8.35ms in the admission
logging phase (four involuntary switches). D2 moving pre1073 falls outside body
velocity component3 of the independent domain and misses deadline during full
current reproof. R57 D2 pre1086 falls outside pose component0. R347 compares
original whole domain, full-source pose symmetry and independently layered
boxes: symmetry covers the old pose miss but not the new velocity miss. Do not
promote additional pose widening or claim this serialization fix closes timing.

Fix the C5 producer before any physical proof: select the closest representable
raw float steering whose exact two-stage wire encoding satisfies the unchanged
absolute and predecessor wire-slew constraints. Retain an already-valid choice
bit-for-bit. Search the finite ordered float lattice; reject an empty feasible
set. The exact source-bound predecessor wire is passed into both prospective
forecast and final nominal selection. The chosen command is forecast anew,
physically certified through rest, hashed and dispatched by the existing sole
normal authority. No post-proof clamp, changed tolerance, send grace or fallback.

Tests: actual r59 failure before fix; positive/negative rounding boundaries,
absolute bounds, empty representable interval, unchanged reachable choice and
adjacent-float optimality; current package tests; full exact-library replay with
new certificate identities only where the command changes. Retained earlier
programs stay immutable. Build, standard runtime and final authority review
remain required. Original25ms windows/receiver250ms/origin130ms/steering100ms and
all model/solver parameters remain unchanged.

R349 all five quantization boundary tests pass. Build r99 all26 and tests r86
all2570records/66groups (source105 included) pass. R350 the new r59 source and
current full proofs admit wire0.4216673970222473 instead of the historical
0.4216673672199249; step0.027671944388944512 is inside the unchanged bound. New
input identities are mandatory. R351 preserves five unchanged-source scenes.
R353 also produces a new source for old r51's rounded nominal boundary; its
original actual before remains late and rejected. No replay sends a command.
R348's mistaken skipped-float datum and R352's optional-field reader failure
are preserved separately; corrected runs satisfy the original criteria. R98
linkage failure was fixed by preserving the existing one-argument public API.
[Evidence](wire-reachability-evidence.json). Next standard dev2-r60.
