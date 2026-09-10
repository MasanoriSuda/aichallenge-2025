# Additional footprint-corner displacement enclosure

2026-09-11 JST. Baseline/rollback e53e3791. M4–M6 remain open.
See [r24 audit](r24-wall-audit.md). Production integration and local validation complete; fresh runtime remains required.

The earliest first common Stop rejects its applied wall at10.979999773 despite
nominal wall clearance. Keeping actual previous249's programme also rejects.
Original pose-box support separates the first enclosing rectangle contact but
fails later11.054999773; minimum−0.012405141m. More rectangle geometry alone
cannot close the full stop. Equivalent world axes r122 worsen it (first
rectangle10.654999773, first
unproved10.734999773, minimum−0.110312126m). Keep this failed hypothesis.

R123initial extra-corner diagnostic did not compile (wrong jet trig API names).
R124corrected it: body bounds bit-identical/native corner samples enclosed,
but one world-axis box adds unused corners and rejects209steps. R125intersects
that additional box with the original oriented enclosure per original contact
cell, but still rejects39steps. Rotation difference cos(new)-cos(old) and its
interval derivatives lose the common angle and widen repeated displacements.

R126uses the equivalent identities
cos(a+d)-cos(a)=-2sin(a+d/2)sin(d/2),
sin(a+d)-sin(a)=2cos(a+d/2)sin(d/2).
The shared native midpoint model supplies d=dt*midpoint yaw rate directly.
Each original input arm/hybrid branch retains independent corner displacement
bounds alongside its unchanged body box; merging carries both conservatively.
At exact rest, corners stay fixed. Endpoint chord bounds are expanded by the
rotation sagitta upper bound radius*max_abs(d)^2/8 for the existing substep
pose sweep. No input value, branch, body bound, margin or integration step is
removed or changed. Every source endpoint body bound matches exactly (1744
components);111616independent native corner values remain enclosed. Original
immediate programme then has0unproved wall-contact steps through rest
11.174999773. Finite samples validate/falsify the implementation; the runtime
claim requires the interval formulas, not these samples.

Implementation: share the existing midpoint map/interval Jacobian with optional
corner propagation, avoiding duplicate wheel/body integrations from the
prototype. Preserve legacy prediction calls and body tube values. An optional
footprint context carries four original margin-expanded local vertex offsets;
streaming proof receives both the original body enclosure and additional
world-axis corner ranges relative to the same observation XY origin. Rest,
all input signs, independent steering, clocks and full programme remain intact.

The applied certificate first uses its original oriented footprint/grid check.
Invalid/out-of-map still reject. For each occupied or unknown contact square,
an additional outward-rounded corner-AABB separation may prove it outside the
same physical footprint population. Whole squares and the existing cell-contact
numerical epsilon remain included; touching/unknown failures do not disappear.
State, speed cap, peers, Follow, complete rest and final identity gates remain.
An auxiliary enclosure alone is never authority. Profile/provenance fingerprints
keep their existing physical meaning; certificate request identity binds the
original footprint and hard clearance. No cached runtime grace or new controller.

Tests must demonstrate unchanged body endpoints; exact rigid-corner native and
substep-sweep coverage across signs/rest/launch/steering bounds and rotated
origins; the captured failing wall population; true wall/cell contact rejection;
missing/invalid auxiliary data; all existing peer/Follow/history/provenance
negatives. Then full package/build, fresh current-header original replays,
record every failed/successful diagnostic in the registry, local commit and
fresh dev2. Runtime25ms and recursiveStop/rest/restart remain unproved.

## Implementation and local verification

The extra enclosure is optional at the numerical API; the applied certificate
supplies the exact original margin-expanded footprint. Both optional validators
must accept; invalid/missing corner context and any failure return no partial
tube. Body-population split/merge order, map equations, input groups and original
body output bounds remain unchanged. The original wall grid scan remains first;
invalid/out-of-map rejects before any auxiliary separation. Peer/Follow/state,
complete-rest, source ceiling and all immutable packet/proof gates remain.

Buildr52 passes26packages in5min0s. Stderr contains existing setuptools
setup.py deprecation messages; no new compiler diagnostic. Package r42 passes
2447records/62groups, errors/failures/skips0 in29.32s. Six new tests cover exact
body equality;2073600 independent scalar corner/sweep values over30initial
heading/signed-speed contexts, acceleration signs, steering limits, rest and
launch; invalid context and both validators; whole occupied/unknown squares,
tangency and map exit; and the captured original947 immediate programme.
The captured fixture uses the byte-identical full r20/r24 grid alignment and
retained region; omitted cells remain Unknown. Original39enclosing contacts
start10.979999773 and all separate through complete rest11.174999773.

Fresh current-header/library r127/r128 replay closes exact947/source245:
10.725639ms,1terminal attempt,2commands,210samples,rest11.174999773,
peer clearance+.680749509m; full proof, production, materialization and join
pass. Original r23/r22/r21/r20/r19/r17 positives and signedr16 join pass.
R22D11024 now accepts the source-horizon first arm (16.299621ms); its former
immediate arm remains covered by the earlier frozen result. Later r24D2990
and r22D21083 still correctly reject their peer populations. The actual r18
history gap still rejects HistoryUnavailable5. Replay times are isolated and
cannot establish live25ms. See [evidence](corner-displacement-evidence.json).

No speculative previous-programme cache, model/receiver-profile/margin change,
new control law or relaxed acceptance criterion is introduced. Rollback is
e53e3791. Next freshdev2-r25 and remaining M4–M6 are required.
