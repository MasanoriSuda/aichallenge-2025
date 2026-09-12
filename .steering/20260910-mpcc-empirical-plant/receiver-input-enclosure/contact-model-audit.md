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
