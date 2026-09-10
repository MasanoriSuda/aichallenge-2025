# Empirical AWSIM shared plant migration

Baseline6b0f1880; previous production MPCC1c4f377e, localization9bca3af6.
The user's2026-09-10reply explicitly accepts empirical simulator acceptance with
unverified guarantees stated, and asks to proceed without routine confirmations.
This supersedes the prior pending question and the requirement to obtain an
external universal transport/contact bound before implementing the candidate.
No physical tolerance, hard wall/peer limit or failed test is waived.

## Selected engineering candidate and limits

Select the fixed four-wheel planar force model with COM longitudinal/lateral
velocity and actual body yaw rate, at the controller base_link origin. Prior
public-input comparisons support this choice over wire=net and the reduced
seven-state force variant. Original single16..35sphysical/contact averages are
the calibration; rejected residual/contact-observer/fitted-coefficient variants
are not reused. Cached-contact epoch is documented model error: its standalone
rollout variant worsened the retained comparison and is not substituted.

Nine states: lateral error, lag error, heading error, COM forward velocity,
progress, semantic desired steering, physical tire steering, COM lateral
velocity, body yaw rate. Inputs remain wire acceleration, semantic steering
rate and virtual progress speed. Semantic desired steering retains the existing
single wire conversion; tire response uses the inspected local actuator law.
New schema/profile identity must reject old seven-state artifacts and warm starts.

At the effective actuation boundary, one native kernel owns wheel drive/rolling,
side force, yaw moment, COM/base_link velocity transform, drag, tire response and
the nominal settled-contact rest branch. The latter uses the inspected Drive
low-speed/input condition and is explicitly an empirical settled-road assumption,
not knowledge of future all-wheel contact. Actual wire/gear/contact replay stays
a separate component observation. Positive input must leave rest; negative Drive
drift uses the restorative law. No generic flat residual or zero-speed clamp is
added after solving/publication.

The existing publication/control-origin offset remains a nominal scheduling
convention, not a guaranteed actual application delay. Longitudinal latest-value
selection can skip packets; source/publication/receipt/application stay distinct.
The shared prefix must use the same input-dependent body law and immutable
published history. Unknown application/contact/terrain and sensor skew form
empirical prediction error; do not call a nominal sweep an unconditional real
vehicle safety proof. Online current-world revalidation and Emergency retain
their actual responsibilities. Explicitly monitor predicted/observed state error
and include its measured values in acceptance, alongside collision/penalty,
wall/peer proof and final-authority provenance. No unobserved case is declared
verified. The model is local2025-derived simulation calibration; real vehicle and
2026official plant bounds remain unverified.

## Implementation and validation boundary

First add the shared native kernel as a nonexecuting library and compare its
fixed-model rollouts against saved inputs, including variable duration, stop,
launch, turning and the original wire/net counterexample. Freeze the candidate
before new simulator evidence. Production connection then replaces all existing
normal prediction consumers in one coherent slice: tangent, physical rollout,
Stop, artifact cursor, prefix, current-state producer, serialization and async.
Retire velocity-affine overrides, yaw-to-steering inversion and prefix-only
response residual from normal authority. Preserve only explicitly diagnostic
historical readers/tests needed to read old evidence.

No production migration is complete until package/native/replay/build pass and
same-model A/B/C/D comparisons, ordinary single and coupled runs have been
evaluated. If a new failure occurs, retain its first decision/artifact/current
world and repair its producer. Continue M4intent/Stop-restart/async, M5repeat
campaign/dev3/dev4/gates and M6same-artifact submission/eval. Empirical acceptance
does not turn an observed failure into a pass. All outcomes, limitations and
necessary local commits are recorded without another routine user approval.

Rollback is a focused revert of the eventual migration slice; never reset the
workspace or alter protected user results, simulator binaries or existing bags.
