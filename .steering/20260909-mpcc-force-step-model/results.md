# Force, current-state and actuator evidence

Observation baseline094941b5, MPCC source1c4f377e. No new controller/model authority.
The generated seven-call probe removes to the original3608method CIL, including
branch/EH targets; no differences. This establishes code preservation, not runtime
timing parity. Original DLL restored; generated DLLs and logs remain under output/.

Single diagnostic1lap44.63533020019531s/penalty0 supplies13455physics records.
Dev2 stops at first moving D1Emergency928/source9.709999782; both bodies3346records.
Actual wheel-force sums match body accumulated horizontal force increments within
0.000541N. Vertical/contact reaction is not wholly measured. All16ground-contact
patterns occur; equal continuously grounded wheels is rejected. Full single force
summary includes post-Finish; moving model comparison ends before59.54s.

The deployed seven-state wire-as-net-acceleration law has single5ms velocity MAE
0.006036569m/s; actual force/body derivative reduces it to0.000698446m/s. The model
has omitted contact-dependent drive, resistance and coupled lateral/yaw motion.
COM is0.3090076605m behind the pose origin. Raw VelocityReport carries COM vx/vy;
its Euler-derived heading_rate has winding spikes and cannot supply body yaw rate.
Calibrated IMU angular velocity agrees with body rate. Odometry omits lateral
velocity and its speed filter introduces a separate lag.

A native moving six-body-state diagnostic retains COM velocity and pose-origin
rotation. Rest with no input, rigid reference transformation, isolated moving
resistance and81tire passivity cases pass. It is not a complete rest/gear/sleep
model. Zero-duration invalid input and very large duration also need hardening
before promotion. Nothing in this header executes in the controller.

Causal rollouts use initial private physics state and prescribed actual applied
wire/tire-angle sequences, without future body/contact measurements. Contact
averages are fitted only on single16..35s; holdout35..59.54s and independent dev2.
At0.5s, single holdout position MAE0.19014->0.05376m, speed0.62952->0.06170m/s;
at1s position0.61905->0.18060m. This supports lateral/yaw states but does not prove
deployable prediction: initial physics state and future actuator output are oracle
inputs. Contact uncertainty, application timing and complete rest remain open.

The actual post-receiver steering actuator is reproducible: float32 timestamp
queue,0.1s delay,0.02s discrete lag,80degree/s slew, effective grip0.7. After the
initial0.3s unknown-queue window,10321single steps have tire-angle maximum error
1.90735e-6degree; dev2each3283steps maxima1.49e-8/2.38e-7degree. These use applied
receiver input, not future measured steering. This does not establish DDS delay
bounds or allow replacing the existing0.13s prediction prefix by an observed mean.

Same-world928 comparison preserves fp5606420468864100205. Nine normal A/B/C/D
and four physical Stop arms all reject; conclusion Unknown. Prototype-body
comparisons and current architecture comparisons use different models and must
not be compared as authority candidates.

This audit uncovered a simulation IMU extrinsic and initial-heading defect.
[The producer repair](../20260909-mpcc-measured-initial-heading/results.md) is the
next committed production change. Complete shared model and M2–M6 remain required.
Failed probe builds r1permissions/r2references/r3Cecil and r4success are preserved.
See evidence.json for hashes, inputs, reports and command logs.

Correction2026-09-10: the prototype pose above is the GoKart Rigidbody origin,
not the controller base_link, which is0.485m rearward and0.05m upward. Its COM
is+0.175992m forward. Those root-origin results remain valid for that point, but
cannot alone justify a controller state-dimension change. See the corrected
[base_link/public-input comparison](../20260910-mpcc-shared-plant-model/results.md).
