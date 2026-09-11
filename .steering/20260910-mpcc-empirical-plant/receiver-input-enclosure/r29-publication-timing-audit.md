# R29 publication deadline audit

2026-09-11 JST. Baseline/rollback f3b444f2. Autonomous M1–M6 continues;
no new production change during timing comparisons.

Dev2-r29 first post-publication crossing is D1decision831 beforeReady:
nominal7.394999834, before7.419999834, after7.424999834, deadline7.419999834.
The new guard detects the crossing; this is not certified publication acceptance.
First moving Emergency is D1decision931,0.14m/s, WP29, wall1789093179.801124316,
pre-publication packet/time rejection. D2first1078,0.16m/s, similarly rejects.
The monitor preserves all violations and shuts down; no six-lap acceptance.
No architecture snapshots were generated because the physical proof itself
passed and the final guard failed later. Exact failing proof reproduction is
therefore unavailable from this run; bracket/timing logs and MCAP remain evidence.

The last required invariant is that the full same-decision proof finishes before
its certified actual publication deadline. Long same-world proof cost was
already sealed in r27/r28; correct clock binding now exposes it at the publisher.
Do not enlarge the25mswindow/250msprofile, backdate history, hide rejected packets,
or reuse old joined boxes without a valid current-state/input proof.

Next bounded timing comparison r186 uses current source/window and original
r27long/r26wall/r24peer snapshots. The numerical Jacobian currently turns exact
zero-minus-zero derivatives into tiny nonzero intervals, so later operations
lose their existing exact-zero shortcuts. Candidate preserves only mathematical
identities0-b=-b and a-0=a for derivative subtraction; nonzero subtraction remains
outward-rounded. This is a diagnostic numerical representation change, with no
physical/input/margin/authority changes. Inspect proof outcomes, enclosure bounds
and timing before considering promotion. SIMD r179 remains rejected/unpromoted.


The bounded r187sign multiplication, r188compiler inlining and r189exact
representable arithmetic identities do not close the deadline; no promotion.
R190compares a numerical architecture: one interval directional derivative for
f(center+t*(x-center)), t in[0,1], over the same complete initial box. Seed each
input derivative with its outward x-center interval and propagate the same
native map. The midpoint value plus that derivative enclosure contains the
image by the mean-value theorem; natural-extension intersection remains. This
loses some dependency precision relative to seven separate derivatives, but
keeps all native modes and physical/peer/rest gates. It is diagnostic only.
A possible use is a cheap complete certificate followed by original refinement
on rejection; that is not yet implemented, and a coarse rejected tube is not
physical infeasibility. Compare original r27/r26/r24 outcomes and all native
inclusion checks before deciding whether the cost/precision tradeoff is useful.


R190one-direction enclosure loses sufficient dependency to reject both previously
acceptedr26requests andr24previous989; no promotion. R191instead preserves all
seven derivative components. For a fixed native map/model/duration, derivatives
evaluated on a larger domain also enclose derivatives at every contained input.
The prototype guesses a neighborhood by five cheap natural-map steps, then
requires exact containment of all six body/steering variables and acceleration
for every reuse. The guess is not a reachable-state assertion. It recomputes
current natural values, midpoint values and every physical sample; only the
Jacobian is shared. Four diagnostic entries are cleared per prediction. No
cross-cycle request/proof authority, input relaxation or production cache is
introduced. Check cost and enclosure precision on the same sealed cases first;
an eventual implementation must own this bounded context explicitly, without TLS.


R191also rejects formerly acceptedr26/r24cases; the enlarged derivative domain
loses too much precision and is not promoted. Disassembly of the original
numerical binary shows interval-Jacobian multiplication/addition remain outlined
functions (about1291/659bytes). Most kernel operations multiply a dual number by
a model constant whose derivative array is exactly zero. An outlined operator
cannot eliminate that known zero work at the call site. R192therefore tests
forcing only existing J algebra inline, preserving expression order, outward
rounding and every box bit; no new numerical approximation, fast math, ISA or
controller setting. Require complete tube bit equality and meaningful timing
improvement before any production change.


R192preserves the recordedr26current/r24previous tube bits but worsens timing
(r24previous19.802→23.894ms; current55.456→64.607ms). It is rejected too.
[Sealed evidence](r29-publication-timing-evidence.json) covers246files,
all seven numerical comparisons, phase-separated live publication events,
original source/binaries and protected-result restoration. No new production
optimization is promoted. Further work must change the proof scheduling/
reuse architecture with exact state/input membership and full current-world
checks; age-only reuse or membership in an uncorrelated joined box is insufficient.


Next bounded r193audits necessary conditions for reusing an actually earlier
certified programme slice. Capture the unchanged numerical predictor's individual
body partitions and paired corner boxes near the new observation source epoch.
Express the new exact observed state and rigid vertices in the prior origin with
outward arithmetic; require both fit one paired partition. A joined box alone
is insufficient. Then compare every remaining channel/sign-group interval and
complete-rest clock under a constant native-time alignment. Compare the existing
full-window rebase with a diagnostic residual window that consumes the prior
packet's elapsed phase (W minus phase), preserving its original latest deadline.
Neither branch grants nominal/current-world proof or adoption. Originalr23long,
r24peer andr26wall pairs are selected before collection; preserve all failed
membership/input checks. A positive diagnostic alone does not establish suitability for production;
the current authorized task still requires full validation.


R193has no paired state/corner membership in any of the three original pairs.
Consuming the old publication phase does fix the future input subset inr23/r24,
but membership still fails; r26actual prior firstpacket was35mslate and its input
subset correctly remains rejected. R194then propagates a fresh current prefix
up to the existing250msreceiver interval. Even excluding the desired-command
coordinate that is overwritten from newly checked channel bounds at every step,
none of the three pairs has full paired partition containment. It is not promoted.
No current-world proof/authority is inferred; negative membership suffices to
reject the proposed shortcut without claiming physical infeasibility.

R195tests a different numerical implementation of the same map. Wheel forces
are affine in body u/v/r and total drive+rolling for a fixed tire angle. Assemble
interval force partials with respect to u,v,r,tire and totaldrive once per wheel,
then chain those five partials to the original seven input derivatives. Preserve
original clamp/reverse/rolling derivatives and every native mode. All value
intervals still run the original shared kernel verbatim. This changes interval
Jacobian evaluation only; no physical model, inputs, step size or margins. It is
a diagnostic hypothesis. Any candidate needs an independent high-precision
Jacobian oracle, full native state/corner inclusion, all negative fixtures and
current-world proofs, build/tests and committed live acceptance before promotion.


R196–198 high-precision checks andr199–202native regressions support promoting
only the same-step force coefficient implementation. R193–194reuse remains
rejected. Rebuilt productionr203–206matches the candidate; build/package checks
pass with a meaningful correction to an obsolete coarse-tube expectation.
[Design and validation](shared-force-design.md) retain all negative fixtures,
original input populations and deadlines. Dev2-r30is the next live check; the
remaining long rejection cost is not closed by offline results.
