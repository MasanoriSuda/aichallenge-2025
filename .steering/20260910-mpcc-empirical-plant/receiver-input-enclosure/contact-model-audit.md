# Contact model and current authority

2026-09-12 JST. Controller8d9810e0, documentation HEAD e866ecf7, build120/tests107.
This audit does not change production. Full M4–M6 remain incomplete.

## New observation

Bounded force-single-r1 uses the existing seven-call observation DLL, with the
original CIL/probe hashes checked before launch. Force/application logging changes
timing; this is not a standard acceptance run. It stops on moving D1Ready950 at
12.154999728. R533 reproduces source632/job945/index2/current wall rejection at
12.519999728 with exact source/domain/actual history/last publication/checks, three
times. Do not conflate this world with uninstrumented single-r6 decision968.

There are1931 force records, no probe errors,711 receiver records and165 direct
applications. R536 binds all applications to exact receiver sequence/source stamp/
float32 values;564 public packet signatures associate the sole receiver withD1.
Observed source-to-apply age spans0.000242447–0.046480135s; intervals between applied
updates0.099999721–0.175136691s. Includes startup/teardown; this is no future timing
bound and does not replace the existing250ms profile.544 received packets before
the final application are overwritten. Source, receipt, application and force
sampling epochs remain distinct.

R531 reuses the physical-force extractor. Its wheel-force sum agrees with actual
accumulated force (moving component MAE3.97e-6N). Its original yaw calculation emits
an invalid-inertia division warning on unselected raw states; no universal force/
yaw validation is claimed. R532's separate807 eligible pre-failure samples and all
native outputs are finite. The physical oracle is unavailable as a controller
input and is never used as a source prediction feature.

## Causal comparison

The current fixed contact weights were produced by averaging IsGrounded over the
old single16–35s moving calibration. Each traction fraction is mean-grounded/4;
cornering is the same duty-weighted physical sprung-mass coefficient. This is not
an instantaneous physical contact state. Producer:
20260909-mpcc-force-step-model/compare_body_models.py:43.

R532 compares the actual built native derivative on identical current body and
actual wire/tire input. Arms: original averages; fixed all-grounded coefficients;
private current-contact/point/sprung oracle. No coefficient fitting. The original
positive-input one-step speed MAE is0.000877900m/s versus oracle0.000137331; braking
0.003153406 versus0.000137444. Thus contact averaging contributes materially to
short-step prediction error in this run. Native planar equations omit3D gravity/
pitch/roll/contact-solver effects; force derivatives and next-step state error
are separate comparisons. No one-step result establishes multi-step containment.

R534 uses497 fixed anchors and prescribed applied wire/tire inputs, withholding
future body and contact in BOTH fixed models. New pre-failure low-speed and old
training/holdout/dev2 windows are separate. All-grounded improves new speed error,
but at1s the old single holdout speed MAE worsens0.111252→0.283845m/s and yaw
0.032699→0.138534rad; oldD1 also worsens. Original fixed averages remain imperfect;
a global all-grounded replacement is REJECTED. Do not select a model by the new
run alone. These are actuator-input-boundary rollouts, not public-state or
closed-loop acceptance. No gains, margins, force coefficients or bounds change.

An exploratory count check also refutes assuming at least two wheels grounded
at every instant: old single16–59.54s awake Drive includes568 one-wheel and71
zero-wheel samples. New force-run pre-failure samples happen to have at least two.
That property cannot be generalized to the old holdout or a future race.

## Next bounded work

The earliest relevant model invariant is source-tube containment of later physical
observations. More seed replacements or an unchanged constant-contact refit cannot
establish it. Before another production change, compare explicitly represented
contact uncertainty/observation contracts on the sealed new and old worlds.
Separate causal public observation from privileged current-contact oracle, and
pointwise contact assumptions from accumulated contact over time. Preserve the
rejected past-window contact estimator/cached-contact models and their revisit
conditions; a new estimator must have a distinct causal mechanism and independent
holdout evidence. Do not invent a positive minimum-contact guarantee from the new
run, add an unexplained residual/margin, or make conditional model proof a physical
guarantee. Use the existing empirical-acceptance authorization and record its
limits; no routine approval checkpoint is introduced.

A candidate needs explicit model/source/async identity, shared QP/native/Stop/
current proof, all original physical and timing gates, negative regression cases,
full build/package tests and new committed dynamic acceptance. A diagnostic-only
model or programme cannot execute. Existing genuine clock failures and all
M4–M6 races/gates/submission work remain open.

## Temporal contact comparison (R537)

The exploratory ground-count check is now reproduced and sealed with a fixed
protocol: the R534 windows, all awake Drive samples, no speed/contact selection,
5/25/50/100/250/500/1000ms windows and separate domains. Old single training has
24 zero-wheel/185 one-wheel ticks; holdout47/383. The holdout's longest observed
all-ungrounded segment is35ms and at-most-one-wheel segment60ms. Minimum mean
grounded wheels in100/250/1000ms windows are0.60/1.28/1.96. These are observed
statistics, never universal contact availability or an accepted Rest assumption.

Forecast comparison uses private previous-step and25/100/500/1000ms past duty,
with future contact only as scoring truth; no fitting. At1s in old holdout,
fixed front/rear duty MAE0.07048/0.06954 versus previous-step0.18952/0.24434.
Past1s improves front0.06574 but worsens rear0.07721. New low-speed and oldD2
benefit on several arms, but there is no uniform forecast replacement. This is
a privileged past-contact observation diagnostic, not a rerun or promotion of
the rejected public least-squares contact estimator. More accurate instantaneous
contact does not itself supply an accurate future contact programme.

[Sealed run, diagnostic payloads and protected restoration](contact-model-evidence.json).
Next separate observation component timestamps from nominal plant error on the
exact saved failure before selecting a new public observer or uncertainty model.
No new standard run is justified by an unchanged rejected fixed model.

## Exact input/model boundary and corrected scoring (R538–R544)

The force-run programme source has aligned pose and velocity at12.004999731.
Its public speed0.091545850 agrees with physical COM0.091545822. At12.144999728,
public/physical speed0.20933230/0.20933235 exceeds source upper0.183793211.
R540 checks30physics instants: every actual wire belongs to the original UNION
of sign ranges, while24speed samples leave its body tube. Maximum speed excess
0.026916949m/s. Twelve tire values also miss by up to2.166e-9rad; keep this
finite-precision/update-epoch limitation, without declaring exact tire containment
or enlarging any tolerance. Initial yaw is a5ms older IMU component, explicitly
held. BodyIMU remains appropriate: VelocityReport.heading_rate is an Euler-angle
difference in the local DLL, not rigid-body angular velocity (plant CIL1249–1290).

R539 uses500fixed anchors with synchronized/held initial components and private
future-contact/geometry oracles. At exact source+140ms, synchronizing initialization
barely changes speed error−0.025650→−0.025638m/s; future contact alone reduces it
to+0.001659m/s. This isolates contact dynamics as a contributor independently of
that sensor skew. Actual future applied wire/tire is prescribed in every arm.
No future physics body is reset, but future contact/geometry is privileged input;
these arms cannot become controller authority.

R541 exposed an error in the diagnostic position truth: R534/R539 scored kart-root
motion against the native model's base_link motion. Production COM offset was
already correct. R542 converts truth with the original serialized offset
(0,0.05000000074505806,−0.48500001430511475)m and rescores UNCHANGED native outputs.
Original position scores are excluded. Original/all-grounded oldholdout1sposition
MAE is0.167555/0.593733m. Speed/yaw scores are unchanged; all-grounded remains
rejected. Future-contact-only position MAE improves to0.077222m in oldholdout and
0.014009m in newforce versus original0.167555/0.057117m. Correctly referenced planar
kinematics with measured velocities has oldholdout1serror0.015703m, not0.343490m;
the previous large number was not evidence of a production kinematic defect.

The new shared diagnostic reader requires explicit kart_root/base_link and one
vehicle identity; it keeps COM velocity separate from pose position. R544 checks
its measured COM/base offsets on the new single, old single and both old domains.
Ambiguous multi-domain reading rejects. Future contact diagnostics use this reader
instead of duplicating the incompatible pose extraction.

R543's setup assertion incorrectly excluded future sleep/Stop; it never executed
native code. R544 preserves those future cases and evaluates453common anchors
with1s of past awake Drive history inside each original window. Fixed preceding
1/5/20/100/200tick contact forecasts receive no future contact/body data. On226old
holdout1sanchors, every past forecast worsens position/yaw: original0.170395m/
0.033656rad versus even past2000.180468m/0.041933rad. Several new low-speed and
short-horizon scores improve. OldD1's1sresult has only one anchor; do not generalize
that mean. No global past-contact replacement is accepted, even with private
observation better than currently available public inputs.

[Sealed diagnostics, corrected comparison and reader](contact-prediction-evidence.json).
Next compare an explicitly evolving contact-mode forecast with the unchanged
fixed model and rejected static-past forecasts. The initial comparison may use
private PAST contact only to isolate forecast capability; this is not a public
observer or production candidate. Training windows, model identity, independent
regression windows, no-future-feature checks and promotion/deletion boundary must
be fixed before execution. An observed contact statistic supplies no universal
Rest guarantee. All normal/current/Stop and original timing gates still apply.

## Forecast and calibration coverage (R545–R549)

Global transition forecasts only slightly improve the common cohort. The old
training has no meaningful acceleration variation and no low-speed samples;
R547's numerical convergence does not identify braking dependence. Speed/lateral
R548 improves new low-speed and oldholdout nominal scores but regresses oldD1.
R549 adds explicitly separated r1low-speed calibration and freezes two models
before a new force-r2 observation. Exact-source speed improves, yaw/oldD1do not
close. No production promotion or physical bound is claimed. Previously inspected
regressions, training overlap and the prospective new world remain distinct.
[Fixed protocol, comparison and remaining gates](contact-low-speed-design.md).
