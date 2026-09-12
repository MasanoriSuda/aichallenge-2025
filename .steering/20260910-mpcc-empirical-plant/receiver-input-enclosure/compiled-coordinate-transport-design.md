# Transport immutable physical Jacobians into numerical velocity axes

2026-09-13. Production/model baseline f2a89c14/8d9810e0; rollback f2a89c14.
User authorizes causal implementation, verification and local commits; no new approval.
M4–M6 remain incomplete. No source change was made during single-r10/R631–R636.

## Observed boundary

Single-r10 D1 Ready999/job996/source660/index0 accepts current physical proof but
rejects the actual before-publication clock12.514999720 after deadline12.509999822.
The callback enters12.489999720, uses17.866918ms wall/17.741134ms CPU, including
16.214674ms current proof. Simulator clock advances25ms. All1142callbacks remain
below25ms(max20.404766), which does not establish actual publication-window success.
551normal sends occur through1140. No out-of-window normal packet, Start or lap.
Normal sends resume after stationary1000; later1016has no due/active programme,
1020source scheduled proof rejects wall and1021normal sends resume. These are
separate boundaries, not one sustained failure. Public max speed0.9889339209m/s.
Runtime stopped; both userJSON and original DLL hashes restored/unchanged.

R633replays999three times: exact source/domain/history/last actual source,
accepted current physical proof and rejected actual before guard.14.215–14.295ms
is offline current proof, not live callback time. R634scopes the same immutable
current query without changing production. Physical population~4.02ms, actuation
population~9.84ms. Physical Jacobian maps recompute104times, actuation661times;
force coefficients alone~0.93ms, corners~4.08ms, overlapping inclusive scopes.
The physical worker cache cannot be used directly in different numerical axes.
R636separately reproduces1020source wall rejection at13.389999709; timing repair
cannot be relabelled as its physical repair or full race acceptance.

## Bounded numerical architecture comparison

This boundary is inside current physical proof for an immutable solved source;
Mission/solver work is outside the measured16.2ms. Hold that source fixed and
compare the numerical execution layer. A is f2a89c14independent per-frame maps.
B transports a contained physical worker map by a fixed invertible T. Source,
inputs, model, hard wall/dynamic/rest proof and actual clocks stay fixed.
R635A14.36–14.42ms vs B11.68–11.84ms,267transported maps,1422candidate membership
misses. Both accept the full current proof and reject the real late before clock;
all immutable identities agree. No runtime promotion from these observations alone.
Caching only tire coefficients could save at most the overlapping0.93ms; it is
not selected. Do not add timer grace, skip a frame/validator or accept a point oracle.

## Transport contract

The worker stores a physical parent P, its exact reference point c, enclosed
native image Phi(c), natural range and outward Jacobian J over P. For query box W
in actuation axes, require the whole outward-decoded T^-1W to lie inside P before
reuse. Merely testing W inside the box enclosing TP is insufficient: the box
loses the velocity dependence of the transformed parent.

Compose the output Jacobian as T J T^-1; transform the point image by T. Transform
yaw-increment derivatives only on input columns. Cartesian translation stays exact.
The reference Tc generally is not a representable point: retain its outward
interval and compute offsets W-[Tc], never use only its lower endpoint. The actual
Tc exists in that interval, so the mean-value bound over the segment in convex P
remains sound. Range and mean-value bounds still meet at the original intersection.

Keep duration, acceleration, model-pointer, whole-query, corner-availability and
numerical-frame checks. Transport only from physical worker axes to the requested
frame; retain exact matching for existing maps and reject unsupported frame pairs.
A missing admissible cached map uses the same native map calculation as before.
No source fingerprint, physical model, input population, proof margin or clock changes.

## Implementation and acceptance

Producer: detail/mpcc_vehicle_enclosure.hpp::centered_step cache selection and
reference-center handling. Retire only the unconditional different-frame miss for
supported physical-parent transport; keep original physical maps and sole validator.
Add native body/corner/swept containment and an escaping decoded-query negative.
The previous blanket no-cross-frame test becomes the unsupported-frame guard;
its exact fresh-result expectations remain. Demonstrate the new reuse regression
against old code, then native/retained/package/build/source tests, five historical
negatives,1986positive,999timing and prospective live acceptance. Keep1020physical
failure separate. Full restart/intent/async/multicar/gates/submission M4–M6 remains.

## Production verification before next live run

R637old implementation fails the new reuse regression only on reference-center
identity (3456assertions); native containment itself remains valid. R638new
implementation passes54native vehicle and162retained tests. The unsupported-frame
negative keeps exact fresh-result equality. Build126all26packages and package113
all2622records pass, zero errors/failures/skips; source/architecture112pass.
R639retains1986full positive and768000native checks. R640current999takes11.726–11.835ms
offline while preserving exact source/domain/history/lastactual identities, original
physical proof and actual late before-clock rejection. R641/R642five historical
negatives and R643same-run1020source-wall refusal remain. This is not live timing
or restart/Start/lap acceptance. Next local commit and bounded single-r11.

[Sealed results and hashes](compiled-coordinate-transport-evidence.json).
