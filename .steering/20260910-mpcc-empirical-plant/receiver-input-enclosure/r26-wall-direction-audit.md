# R26 wall direction audit

2026-09-11 JST. Baseline and rollback: `963f9aa3`. Autonomous M1–M6 work
continues. No integrated acceptance. Production stayed unchanged through r145; its validated
directional-support producer is now being integrated and tested.

Dev2-r26 first fails at D1 decision 928, 1.20 m/s, WP30, wall time
1789081322.334845275. Decision time is 9.909999778, control origin
10.039999778. Final inspected source 131 is distinct from the preceding actual
publication 927/source143. Both captured requests and actual publication clock
are preserved. D2 decision996 fails 2.1404 seconds later; do not attribute its
peer failure as the original cause. Protected result JSONs were restored.

Previous D1 927/source143 passes full proof, production, materialization and
join (14.327656 ms). Current 928/source131 rejects after four terminal attempts;
the final applied wall failure is at 10.479999778. Actual source143 reconstructed
from the original current observation and publication clock also fails. Its
normal first packet is `[9.909999778, 1.3295953273773193, .057717978954315186]`.
The nominal published Stop accepts and materializes, but its full current join
rejects the same wall time. That first braking packet is
`[9.909999778, -3, .06708534061908722]`.

Same-world A/B/C/D/G ordinary architecture arms all reject (OSQP maximum
iterations, including wall-refined candidates). Complete-rest Y first rejects;
its existing 5-second second candidate nominally accepts in 31.9097 ms.
Neither all-arm rejection nor finite sampling proves physical infeasibility.
The accepted Y result has no complete applied certificate and cannot execute.

The first failed D1 callback is 150.382308 ms: primary 101.578, lattice 28.438,
Stop join 13.273 ms. Emitted whole-period telemetry has D1 p99 40.92893175 ms
and 161 adjacent overrun pairs; D2 p99 34.99906258 ms and 58 pairs. These
include startup/teardown; the final incomplete telemetry window is unobserved.
The 25 ms condition remains unmet.

## Sealed comparisons

| Tool | Changed diagnostic dimension | Result |
|---|---|---|
| r140 | Rebind the previous actually certified common programme to the original current observation | Nominal accepts; full wall rejects 10.624999778. Retention alone is insufficient. |
| r141 | 128 separating directions, whole original body/corner ranges, actual published Stop | 121 cell/time comparisons remain unproved, first 10.479999778, minimum gap −.080671464 m. |
| r142 | 2/4/8 speed subdivisions, retaining all original inputs and branches | 63/61/60 joined wall steps remain unproved; per-branch checking also fails. More speed bins alone are insufficient. |
| r143 | Same directional support on the original previous common programme and captured first-packet constant-steering Stop | Both clear through full rest 10.764999778; minimum gaps +.0083514495 / +.0097503195 m. Geometry diagnostic only. |
| r144 | Full native certificate with grid and midpoint vehicle axes as proposed separating directions | Actual143 normal and published Stop still reject. Eight directions alone do not close the full authority path. |
| r145 | Include original observation-frame axes among the support directions | Actual143 normal fully accepts in14.420313 ms, one terminal attempt, peer+.729321539 m, rest10.764999778. Its immutable Stop materializes and fully rejoins in12.728067 ms. Separate generated Stop remains wall-rejected10.479999778. |

R141/r142 each check 1,142,784 independent native body/corner/interpolated-pose
values within the original ranges, with no sampled contact. R143 checks
1,105,920 values for each programme, no sampled contacts. Finite samples are
regression observations, never the population certificate. In r142,
`body_component_matches` counts finite components, not identical ranges.

R143 establishes an unused-rectangle-corner effect for those two programmes,
but r141 establishes that not every generated Stop is separated by this
representation. Preserve that distinction. No speculative programme cache,
new fallback, numerical partition count, input profile, physical margin,
model, rate, timeout or acceptance relaxation is justified by these results.

## Next causal slice and acceptance

The body XY ranges live in the original observation heading frame. R145 tests
that frame's axes as additional witness directions, preserving coefficients
before multiplying the same coordinate ranges. A witness is valid only when
outward interval support excludes the entire contact square, including the
existing epsilon, from all original footprint vertices. Direction choice is
not itself a certificate. Original wall sampling, out-of-map rejection,
state/speed bounds, CA1 peers, Follow, full rest, immutable programme and final
packet gates remain required.

Before validated production commit: reproduce full actual143 normal and its Stop
materialization/join; freeze the exact failing cell/body/corner ranges; add
occupied/unknown/contact/tangent/invalid geometry negatives; preserve original
native input coverage; build and package tests; replay earlier positive and
negative worlds; review timing and source identities; record evidence and
local commit, then run fresh dev2. Actual publication clock and M4–M6 remain
open after any local geometry repair. Rollback is `963f9aa3`.


## Producer change under validation

The wall producer retains its original enclosing-rectangle query and map-exit
rejection. For each queried contact cell, it now accepts an outward-rounded
strictly positive separating support from either of two independent original
whole-population enclosures: body pose or propagated rigid vertices. Proposed
axes are the original observation frame, midpoint vehicle frame and grid frame
(12 directions). The old world-XY corner test remains the fast path. Translation
coefficients are combined in the original frame before multiplying XY ranges.
Every footprint corner includes the original dimensions and wall margin; cell
support includes its whole square and the existing1e-9 contact epsilon.

No dynamics, population binning, packet, programme, rest horizon, peer or Follow
rule changes. No partial tube escapes a failed proof. The replaced rejection is
"contact with the outer rectangle implies contact is unproved under every
available enclosure"; there is still one common full certificate and one normal
publisher. R145 leaves the separately generated turning Stop rejected. Its
success belongs to the unchanged existing constant-steering normal candidate
and the exact Stop programme materialized from that candidate's certificate.


## Local validation

Buildr54 passes26packages in23.8s, with only existing setuptools deprecation
warnings. Package r44 passes2451records/62groups in29.74s:0errors/failures/skips.
The frozen originalD1928first contact tocell476325 still fails old world-XY
corner separation. The same original constant first-packet programme now
clears every complete response through rest10.764999778. Every body endpoint,
body sweep and corner sweep is bit-identical to the original unchecked tube;
no input/response was removed. Rotated occupied/unknown contact and tangency,
map-axis conventions, margin and invalid geometry negatives pass. Existing
full applied wall/map-exit/peer/Follow/first-packet/identity tests remain intact.

Current-library native146–148passes the earlier positive cases and negative
realr18history gap. Actual143normal accepts in14.822499ms with1terminal attempt(s),
its immutable Stop materializes and rejoins in13.058830ms, rest10.764999778, peer+0.729321539m.
The independently generated publishedStop continues to rejectwall10.479999778,
as required by its unresolved full population. This is not programme caching
or a new normal candidate: the existing constant-steering candidate now obtains
its complete directional wall proof. Captured source131also passes fullproof/materialization/join in13.743805ms;
laterD2996stillpeer-rejects12.954999728. Sources remain distinct. Earlier
r22current replay costs25.140337ms andr23previous44.513635ms; timing acceptance
is open even when the complete proof accepts.

See [evidence](oriented-wall-evidence.json). A local certificate is not live25ms
or race acceptance. Commit this slice and run freshdev2-r27. Actual publication
clock, full dynamic Stop/restart and allM4–M6 remain open. Rollback963f9aa3.
