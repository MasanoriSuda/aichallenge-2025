# R28 wall envelope audit

2026-09-11 JST. Historical diagnostic checkpoint at baseline `50486fcf`;
its root-cause uncertainty below is superseded by the
[publication-window investigation](publication-window-design.md). No production
changes were made during the envelope experiments recorded here. M4–M6 remain open. Same-world materialization reuse is locally verified
and observed live; it does not establish integrated acceptance.

Dev2-r28 first moving failure is D1 decision919, 1.17m/s, WP30, wall time
1789085739.685526745. Inspected source131 is distinct from actual publication
918/source137. The previous request is tagged `invalid` by the capture's inspected
source association, and that tag is retained. Its own complete native proof
accepts; the immutable identity also matches the separately captured actual
publication137. These associations must not silently relabel the captured131.
Current now/control are9.949999777/10.079999777; previous9.919999778/10.049999778.

The normal131 request and explicit current-world rebind of actual137 both reject
with outer `steering-unreachable` and inner applied wall reason6. The final tested
Stop reference rejects at10.544999777; the separately generated published Stop is
nominally accepted/materialized but its full join rejects wall at10.579999777.
There is no all-method infeasibility certificate. A/B/C/D/G comparisons reject
solver feasibility; Y first rejects the solver, and its long candidate solves but
rejects the original wall proof atstage646. Exit4 is recorded for both comparison
commands. No constraints or candidate limits were relaxed.

Before the first failure, materialization reuse is observed17times inD1 and69inD2,
all `numerical_tube_reused=1`. LaterD1materializations are postfailure evidence.
D1decision834 still spends170.414ms (lattice128.255/output36.695ms); its native
first proof91.852026ms, materialization11.036810ms, reusedjoin15.600417ms versus
fresh91.503732ms,1086checks. The first-failure callback919takes219.651ms,
primary123.316/lattice41.144/Stop24.931ms. Shutdown starts1789085746.355165.
Protected user JSON hashes are restored. Aggregate callback metrics include
startup and postfailure/teardown and are not race acceptance.

## Rejected isolated hypotheses

R162retains the exact original input profile/population, model, margins and
actual publication identity. The previous certified programme, rebound to the
new observation, rests at10.789999777 but cannot separate cell476325 at28times:
first10.649999777, worst−.017761815m. The current captured-first constant-steering
Stop rests10.809999777, with42unproved times, first10.599999777,
worst−.029541495m. Existing production directions plus128diagnostic directions
were checked.128native paths each have zero contacts and all1,044,480/1,069,056
checked body/corner values are enclosed. Sample success is not a full proof.

R163andR164fail during diagnostic source generation (multiline declaration,
then overloaded function matching), before compilation. Logs/scripts are
preserved; neither is a numerical result. R165correctly separates the full
six-argument predictor and adds independent corner coordinates in the initial
heading frame. Original body ranges are identical, native values are enclosed,
but minimum gaps remain−.012201602/−.017978922m. R166uses fixed world diagonals;
gaps remain−.011150150/−.016888901m. Neither coordinate-only trial is promoted.

R167compares2/4/8closed forward-velocity intervals with every original rest/signed
branch retained. Prior-programme minimum gaps are−.017761815/−.014911732/
−.013170536m; captured-first constant gaps are−.029541495/−.025824229/
−.023459084m. Full current-world programme joins still reject. All sampled values
are enclosed. More subdivision alone is not a fix and is not promoted.

R168compares seven future steering targets, preserving the exact captured first
normal packet, original minimum acceleration thereafter, original maximum slew
and whole input population. Targets current(.0837423),0,−.1,−.2,−.3665191,+.2,
+.3665191all fail full-population directional separation. Positive+.2/+.3665191
also produce admitted native wall-contact witnesses (9,809/14,352sampled contact
checks, first10.629999777/10.624999777). Other targets' sampled paths are clear,
which is not a proof. No nominal/canonical production authority is inferred from
these direct programme diagnostics and no extra live candidate retry is added.

## Next bounded investigation

Coordinate-only projection and more speed bins do not recover enough dependence.
Test an additional correlated state enclosure with the same shared interval
Jacobian/native map and every original input/hybrid branch. Preserve original
body/corner ranges and physical gates; an additional sound support bound may
supplement them. Explicitly account for rounding, nonlinear remainder, branch
union and any generator compression, and check independent native inclusion.
Do not promote sample-only clearance, add margins/grace, drop inputs, or claim
physical infeasibility from the failed candidate list. Actual publication timing,
continuous deadline overruns, live multi-tick Stop/restart and allM4–M6 remain.

## Correlated enclosure diagnostics, not promoted

R169 adds affine forms using the shared interval Jacobian, explicit nonlinear
remainder, branch union and a32-generator cap; discarded columns become an
independent box remainder. Original body ranges remain identical;8,181,760/
8,374,272native body/vertex/projection checks pass. It does not improve wall
separation and costs33.532219/32.049610ms for its additional prediction.
R170 preserves generator identities across branches and explicitly rounds new
input radii outward. It still does not improve separation (35.381560/33.639373ms).
R171 adds conservative forward-speed conditioning by eliminating one generator
and retaining its error; no physical branch/input is discarded. It accepts1,184
of1,461conditioning operations in the first programme (the second counters are
cumulative), but wall support still does not improve;49.624597/48.476129ms.

R172 measures why. At the first failed previous-programme cell, one stationary
branch has4.054359m of independent X-error width versus the original.395502m
physical X width. Its yaw error is.522015rad versus original.057211rad. Repeated
branch unions and discarded independent errors make this affine representation
far weaker than the existing box, despite containing all sampled points. It is
not a fix and has no production adoption boundary yet.

Next test a fixed invertible linear coordinate template, whose branch union is
an interval hull in one common basis. It must use the same native map/interval
Jacobian, carry all original hybrid branches, account for center/coordinate
rounding and compare original body ranges/native projections. This targets the
measured union-error growth rather than changing physical safety requirements.


## Fixed-coordinate, input search and active-state results

R173 fixed invertible eight-coordinate intervals retain the original body ranges
and all sampled values, but the prior/constant wall gaps remain the original
−.017761815/−.029541495m. Additional predictions take18.378595/18.128177ms.
Its original generated manifest incorrectly describes affine generators; the
archived source is the fixed-coordinate implementation. R174 records the correct
meaning and the remaining wide transformed coordinates: at the first cell,
stationary yaw−kappa*X is[-.029388184,.008448323]rad, and its support gap is
−.011002762m versus the stronger original−.000431336m. Neither is promoted.

R175 performs a bounded native adversarial search over the original input unions,
with12seeds and5,533native trajectories per programme. The best signed
rectangle/cell separation is+.003297801m for the previous programme and
+.003853943m for the captured-first constant Stop. All seeds reach the same
extreme-input path; its closest times are10.769999777/10.789999777. Every sampled
input is admitted and every endpoint remains inside the original body tube.
These positive3–4mm sampled gaps neither prove the whole population clear nor
justify reducing margins. The search is preserved as inconclusive.

R176 retains closed X intervals for stopped as well as moving states, with8/32
position bins and the original speed bins; corner endpoints also intersect an
independently computed body-position enclosure. It still rejects, with prior
minimum−.017693282/−.017458449m and constant−.029521057/−.029485556m.
R177 changes the first packet to the original minimum braking, retains its exact
captured steering, then uses the seven original slew-limited steering targets.
Every word fails the full wall check;+.2/+maximum also admits839/5,159native
contact checks. These direct words have no nominal MPCC or publication authority.

R178 separates two active internal coordinates. Eight speed bins×eight tire bins
reduce prior/constant gaps to−.008261996/−.013990653m. Eight speed bins×eight
yaw-rate bins give−.010435136/−.017685208m. All native inclusion checks pass;
both full programme joins still reject. Refining these unchanged arms again is
not justified without a new causal or representation hypothesis.

## Observation and publication chronology

Read-only MCAP extraction uses only dev2-r28/D1/source9.3–10.35. It records440
messages, including21IMU messages at50ms source intervals,53Odometry messages
and32control commands. There is no missing IMU publication in that interval.
The source9.899999778 yaw rate.16232652962207794 is explicitly held at the
pose9.944999777; the next IMU9.949999777 is.16393277049064636. The latter is
future relative to the initial pose, and its bag arrival cannot prove it was
available to the controller before the callback. This is not an established
root cause and no changed observation reconstruction is adopted.

The command stamped9.949999777 is recorded atwall1789085739.679248, while the
same-stamp IMU is recorded at1789085739.465786. Their receipt separation is
213.462ms. It is evidence of a delayed command, not an exact measurement of
controller entry, publisher call or Unity application. The source9.3–10.35
command gap maximum is214.999995ms; original callback919is219.650944ms.
The actual publication-window proof and consecutive deadline-overrun contract
remain open independently of the wall-enclosure failure.

All r162–r178 results, unsuccessful setup attempts, source scripts, exact
snapshots, native binaries, logs and this run's MCAP hashes are in
[r28-wall-envelope-evidence.json](r28-wall-envelope-evidence.json) and the central
experiment registry. No production change follows50486fcf at this checkpoint.
Next: identify a producer change that restores full current-world Stop viability
and actual publication timing; retain Unknown for all-method feasibility until
there is a bounded physical certificate. M4–M6 remain unfinished.
