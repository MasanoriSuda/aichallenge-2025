# Nonlinear source state and correction design

2026-09-13. Authorized autonomous M1–M6 continuation; local commits, no push.
Baseline58436f9f; normal control baseline9c522a95. Scope is the canonical source
trajectory proof and its existing numerical correction. Current/retained programme
velocity semantics remain a separately tracked consumer obligation, not implicitly
validated by this source change.

## Causal evidence and invariant

R716/R717source537 is a real r18Rejoin world. A stopped-steering tangent leaves all
QP inputs and the racing objective free. The old complete comparison accepts one
solution, although exact final speed is2.317426407 versus source cap2m/s and QP final
speed0.008201322. The exact lag also exceeds the original3m state box at stages18–20.
An added speed check with old disconnected affine-state correction hits wallstage529.
Using one coherent native rollout for post-refinement state selection instead passes
speed and original wall/dynamic/Stop proof after one existing correction. R717native
stage velocities match the dense physical adapter exactly. This is offline evidence,
not production or race acceptance. Initial preparation generation is not yet promoted.

The earliest missing source check is between exact nonlinear reconstruction and
accepting a solved source. The physical adapter exposes dense pose/velocity but not
all nine exact stage states; source acceptance checks the physical pose/declared rest
while original state boxes are checked only on the affine QP states. The original
post-refinement path selects those disconnected affine states again as tangents.

A source accepted for normal planning must retain the unmodified solved controls,
original identity and semantic x0, and validate the independently reconstructed executable-prefix state
knots against the same declared state boxes with the original numerical row tolerance.
A failed numerical solve remains a failure. Exact swept wall/peer/terminal proof is
still required after state feasibility. Approximate geometry rows are not a substitute
for the final physical map certificate.

## Bounded implementation slice

1. Expose exact native stage states alongside the physical adapter result. These are
   diagnostic/numerical data; no new artifact, Store or command authority.
2. Validate original source state boxes on those exact knots, reporting the first
   state/element/value/bounds/tolerance. Initial x0 remains the original semantic
   observation. Keep endpoint constraints distinct from continuous physical limits;
   do not invent interpolation semantics for changing limits or extend the horizon.
3. Route an actual nonlinear state violation through the existing maximum-three
   correction path. Use the same reconstructed state sequence as the tangent input,
   preserving existing numerical state/input selection, objective, hard rows and
   cold-dual bootstrap. Retire reuse of disconnected affine states in this path.
4. Require exact source validation even when no wall refinement happened. Preserve
   full rest, physical proof, source/command identity and existing failure capture.
5. Add a focused failing source/adapter regression, run existing focused suites,
   replay the original wall/velocity positives and negatives, then full build/package.
   Compare additional saved worlds before any stationary preparation integration or
   fresh live campaign. No source seed population change belongs to this slice.

## Consumers and limits

The participant ROS names/types, Domain topology, launch, submission, evaluation JSON
and artifact snapshot wire schema are unchanged by returning native stage states.
CertifiedPlan already retains solver_source_snapshot; source_horizon_velocity_ceiling
uses it for long applied programmes and binds its minimum future positive cap into
certificate/provenance. That does not establish equivalent bounds on all short
bootstrap, Stop, current-domain and shifted-cursor paths. Audit those mappings before
claiming global velocity enforcement. Old snapshots must not be relabeled as live
current authority or new same-source acceptance.

Rollback is the source slice after58436f9f. Do not relax tests, state boxes, model,
solver settings, timing windows, certificate margins or Recovery guards. Passing
this source slice does not close the wall source supply, preparation generation,
post-Rejoin worker latency, r80D2actual send deadline, physical observations or M4–M6.

R723retains initial test failures:5source/mailbox checks expose result_valid's old
assumption that physical checks occur only after wall refinement; require the proof
on every solved source. The horizon-overrun negative is masked by earlier artifact
construction, so reject its original impossible publication interval before physical
construction and retain its exact reason/duration fields. One new matrix-equality
test used Eigen isApprox on infinite bounds; replace it with exact element equality.
No expected physical rejection or safety assertion was relaxed. The native state-box
check covers the declared executable prefix, while a requested correction rolls out
all planning inputs to select one continuous numerical state sequence.

The architecture external-primal oracle has the same native-state obligation. R726
is a test-fixture setup failure (no optional assembly_request), retained separately.
R727 removes that fixture assumption and reproduces actual old false acceptance:
coasting from2m/s exceeds first-knot1m/s although the affine velocity is invented as
0.5m/s. New verification uses semantic state bounds for raw physical oracles and
the exact recorded state-box block for affine policies, respecting their explicit
wall-bucket omission. Raw lateral support still belongs to exact wall proof. R729
preserves both R715right preparation positives and original/left wall negatives.
Source contract tests required the obsolete disconnected helper by name; update that
assertion to require coherent native correction and the new mandatory state proof,
keeping all existing bootstrap/problem/ordering assertions. R728native179pass.

Validation sealed R731: R723initial fixture/result-valid failures; R725core129pass.
R726optional-assembly fixture failure and R727actual old oracle false acceptance
remain separate. R728native179/source118pass; build134all26; Summary: 2643 tests, 0 errors, 0 failures, 0 skipped.
R724new source guard/coherent correction retains original537wall144 and drive4
solver negatives; diagnostic right-drive2passes complete proof. R729fixed four raw
controls preserve two right positives and original/left wall negatives with new
semantic state bounds. R730additional430/622/527worlds retain solver/wall rejection;
unsupported comparison arms remain inapplicable, not infeasibility. Two source
text-test failures (old helper name and incorrectly restricted inspection span)
are retained; native safety assertions unchanged. [Validation](nonlinear-source-state-validation.json).
Next fixed-source single-r19; candidate preparation and current/retained consumers
are not silently promoted by this source fix.
