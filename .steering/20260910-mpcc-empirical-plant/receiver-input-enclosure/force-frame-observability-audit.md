# Force frame, conditional contact and public state observability

2026-09-13 JST. Baseline8d6578a4; controllerfe976b39/build121/package108 and
algorithm8d9810e0 remain unchanged. R576–R582 are offline, no new live run.
**No model/observer promotion; all M4–M6 remain incomplete.**
[Payload hashes and validation](force-frame-observability-evidence.json).

R576 retains11140current force rows. Its planar current-contact derivative agrees
with the original native output to7.106e-15. Substituting full point velocity,
ground directions and full inertia separately locates missing lateral/rotation
terms. Actual ground-direction wheel forces reconstruct captured wheel forces
closely; these are private current attribution inputs, not future observations.
Adding gravity alone remains unsupported because later normal/contact constraints
are outside the recorded accumulated wheel force.

R577 identifies the cached ground direction: Before tire minus the preceding body
yaw change approximates WheelHit direction in the current body frame. In old
training the original front angle errors0.00343/0.00388rad fall to about1.3e-5rad
with Before tire minus r*5ms. Rear directions have the same preceding-frame
rotation. This first-order approximation is not exact 3D physics. The local CIL
sets collider steer then reads GetGroundHit before applying wheel forces; it does
not justify treating the returned direction as newly updated tire in body axes.

R578 retains542old/r2anchors and exactly reproduces14310baseline predictions.
One initial force-r2anchor has no preceding tire and cannot run cached candidates;
all original anchors remain, and comparisons use541common supported anchors.
Original/frozen mixed contact laws are crossed with original RK2, cached-frame
RK2 and a planar held-force step. Oldholdout1slateral error with the original
contact law falls0.05586→0.02995m/s for cached RK2, but yaw/speed and oldD1 issues
remain. Neither phase-only nor step-only change is promoted.

R579 tests the [prespecified single conditional-contact structure](conditional-contact-force-design.md).
It adds absolute/side-signed lateral slip-force demand to the prior contact law,
using exactly3800old high-speed plus400r1low-speed training rows. Both axle fits
have rank7, scaled conditions9.10/11.12 and optimizer convergence. All42876R578
predictions reproduce exactly. The added law does not repair oldD1 and worsens
oldholdout: cached RK2at1s has position0.183831m/yaw0.044669rad/u0.160192m/s error.
This fit is rejected. Do not continue feature/window scans or run a new acceptance
race with an already rejected candidate. R3 was not used to fit this model.

## What the public messages actually observe

R580 extracts13048messages from the retained force-r3bag. /tf_static is absent;
we use the existing simulation extrinsic configuration (yaw+pi/2) as a declared
interpretation and do not claim the recorded runtime TF broadcast was verified.
Source stamps and recorder receipts are separate from controller callback receipts.

From first physics observation through last normal1148, R581 matches182IMU and
454odometry samples, with no source-stamp mismatch. IMU attitude agrees with the
physical body attitude to maximum9.193e-8radper rotation-vector component. Roll/
pitch are therefore present in the public IMU. Odometry attitude largely removes
them; its vertical twist is always zero. Public pose-height secants differ from
physical base_link height secants by maximum0.372223m/s. Do not substitute the
filtered odometry fields for direct body heave/roll/pitch.

Public roll/pitch gyro with private lever height reduces mean horizontal point-
velocity error, but worst lateral error grows0.018935→0.037127m/s. R582 explains
why raw gyro is not a direct instantaneous wheel-velocity observation: the local
ImuSensor computes quaternion change across each5msphysics step, then publishes
at20Hz. Across181supported samples it agrees with finite pose rotation to maximum
[1.020e-6,1.863e-6,4.304e-5]rad/s. Its difference from instantaneous Rigidbody
omega reaches[0.179361,0.027343,0.006158]rad/s. One first sample has no preceding
rotation and remains unsupported. This is a measurement-definition difference,
not evidence of an IMU publisher or extrinsic defect. The data do not identify
which internal integration/contact-projection operation produces the difference.

## Continuation

The next producer work must represent the measurement being received: public
finite pose rotation versus the instantaneous angular state used by the wheel
model, along with the already established component-epoch and input-application
boundaries. Preserve raw full-message support and derived-state provenance; do
not insert IMU roll/pitch directly into instantaneous point-velocity equations or
rename the measurement as exact momentum. Current contact geometry/latent dynamics
and a deployable uncertainty model remain unresolved. Use a bounded deterministic
comparison with original negatives before changing production. Any model change
must update scalar/AD/interval/QP/native/Stop/current/async semantics and identities
together, followed by build/package and fresh dynamic acceptance. No permission
pause is required; no unvalidated model or new normal fallback is installed.
