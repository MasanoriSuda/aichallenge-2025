# Conditional contact-force comparison

2026-09-13 JST. Baseline/rollback8d6578a4; controllerfe976b39/build121,
model8d9810e0 unchanged. This is a bounded offline model hypothesis, not a new
normal authority. M4–M6 remain incomplete.

R576 preserves11140force rows and reproduces the native current-contact
derivative to7.106e-15. Full point velocity and ground directions explain much
of the remaining lateral force. R577 identifies the cached contact direction:
Before tire minus body-r times the preceding physics step approximates measured
wheel directions far better than After tire. Public body-r/tire are available
state variables; private contact, sprung mass and full point velocities used in
that attribution are not deployable future inputs.

R578 compares542retained anchors, reproducing14310original predictions exactly.
One first-force anchor lacks Before tire and remains explicitly unavailable for
cached-frame candidates. Both a cached-frame midpoint and a held-force planar
step are compared without a contact refit. Lateral errors improve, but low-speed
forward errors and oldD1/yaw regressions remain. Neither isolated change is
promoted. Do not repeat those unchanged arms as a completed repair.

Next R579 tests one additional contact-law structure. Preserve the R549 training
windows exactly: old single16–35s awake Drive u>=0.5 (3800rows), plus force-r1
8–10s awake Drive (400rows). No old holdout/dev2/r2/r3 data enters training. Keep
all fixed original and R549 coefficients unchanged as comparisons. Add two
features to the existing mirror-tied per-axle five-feature contact logit:
absolute and side-signed all-grounded lateral wheel-force demand divided by g.
Compute demand from the same predicted u/vy/body-r, Before tire, cached-frame
angle and unchanged physical wheel coefficients/geometry. This uses lateral
slip already present in the nine-state dynamics, which the earlier contact law
omitted; no private future contact, force, roll/pitch or body feature is allowed
in rollout. It does not estimate a guaranteed contact bound.

Fit this single seven-feature law, record rank/conditioning/optimizer result and
coverage, and freeze its coefficients before scoring existing known regressions.
Do not scan windows, select per-scene coefficients or fit residual safety margins.
Cross it with the same three integration arms and all542retained anchors, keeping
missing Before support explicit. Retain all per-world/horizon/component errors;
oldD1/regression/rest cases cannot be pooled away. These are inspected diagnostic
comparisons, not untouched validation. Keep R3 separate for a later fixed check.

Promotion requires more than lower average error: a causal public state producer,
explicit physical force-frame semantics, all scalar/AD/interval/QP/native/Stop/
current/async consumers and immutable identities updated together, independent
derivative/enclosure checks, original unsafe physical/clock replays, build/package
tests and fresh dynamic acceptance. Preserve single certified normal authority.
Input application uncertainty, raw/derived observation support and rest/restart
remain separate unresolved boundaries; do not hide them with this contact fit.
