# Shared plant inputs, conditional actuator law and corrected pose origin

Continue9bca3af6; MPCC still1c4f377e. No new production authority/model selected.
M1–M6 remain open; these are required comparisons, not integrated acceptance.

## Corrected comparison boundary

A material diagnostic error was found: the earlier force/body comparisons used
Rigidbody/GoKart root as pose origin. The controller uses serialized base_link at
Unity root-relative(0,0.05000000074505806,-0.48500001430511475)m. Its orientation
is unchanged. Thus COM is+0.175992353782509m forward of base_link, not−0.309m.
Earlier root-origin comparisons do establish wire/force mismatch but cannot alone
establish the necessary controller state dimension or its lateral-motion error.
No production model had been changed from those observations.

Using the serialized Environment1298 MGRS offset, recorded actual GNSS-node replay
and physics-derived base_link agree at51same-source points per Domain: maximum
XY component error D1(0.003855,0.001997)m, D2(0.004012,0.002130)m. No fitted map
translation. Vertical geodetic/height conversion is explicitly excluded. This
independently validates the measured-heading/lever-arm repair's horizontal origin.

Final comparison `base-link-models-r1` uses the correct pose origin, public initial
IMU/VelocityReport/SteeringReport, fixed original physical/contact averages and
prescribed future published commands as exogenous inputs. Future body/contact/
actuator measurements are scoring only. Sensors up to35ms old are held; source
and local receipt are both checked. Longitudinal input is nominally immediate
publication, not actual10Hzapplication; no latency guarantee or promotion follows.

Single35..59.54s holdout, position/speed MAE at0.5s:

| Model | Position m | Speed m/s |
|---|---:|---:|
| Existing kinematic yaw and wire=net |0.194635|0.628518|
| Same seven-state yaw, wheel force with zero base_link lateral velocity |0.095839|0.116810|
| Dynamic COM lateral velocity and yaw rate |0.056545|0.066474|

Dynamic1s position MAE0.193661m/maximum0.675902m; speed MAE0.114788m/s.
Independent pre-E dev2D1/D2 dynamic0.5s position MAE0.012659/0.033446m; speed
0.044251/0.119609m/s. D2short0.1s position happens to favor old0.001306m versus
0.002488m, so do not claim every metric improves. Adding states is supported as
one hypothesis, not a proven necessary dimension or accepted physical envelope.

## Other fixed comparisons retained

- `public-inputs-r1` and its root-origin scores are retained, now superseded for
  controller pose conclusions. Holding only the last past command is separate
  from prescribing future publication inputs. Do not call the latter history-only.
- `contact-estimate-r1`:0.5s past public sensor window, two bounded axle-contact
  weights, no future data; all490single anchors identifiable, two early missing
  anchors per dev2Domain. Single0.5s position MAE0.066253->0.080008m and1s
  maximum0.739249->1.449915m worsen. D1also worsens; D2improves. Reject generic
  promotion; do not retune the observer against the failure. Root-origin scores.
- `body-identification-r1`:four bounded effective axle weights, training16..35s,
  93initial states and0.1/0.25/0.5s horizons, actual applied inputs, future body
  solely supervised targets. Eight optimizer evaluations reduce training RMS
  0.033603->0.032273m equivalent. Frozen public-input holdout has little mean
  improvement and worse1smaximum0.774864m; both dev2speed errors worsen. No
  transfer of these fitted coefficients into production. Root-origin scores.
- `steering-source-r1`:propagating the old SteeringReport to the IMU source epoch
  with past published commands does not resolve the error (single0.5s position
  0.066253->0.067273m). This is not permission to fit a delay. Actual receive
  timing and publication timing differ. Root-origin comparison preserved.

## Native actuator/contact contract

`actuator_contact_contract.hpp` transcribes the inspected post-receiver CIL into
an isolated native diagnostic. Queue delay0.1s, discrete lag0.02s, slew80degree/s,
grip0.7; physical Drive input bounds+1.37/−8m/s², rolling0.37m/s², minimum force
interval0.01s. Limits reproduce the actual plant, not post-solve command repair.
Contact-qualified sleep needs all four wheels grounded, |v|<0.02m/s, nonpositive
Drive input. A positive launch input exits sleep. Below-zero Drive speed changes
brake response to restorative acceleration. Zero speed alone does not prove sleep.
Future/malformed queue history and nonfinite/backward/duplicate physics epochs
fail closed. Reverse/Park are not silently treated as Drive.

`g++ -std=c++17 -O2 -Wall -Wextra -Werror .../test_actuator_contact_contract.cpp`
and native r2 pass delay-boundary, slew, source-order, malformed history, all16
contact/rest/brake/launch modes and invalid input checks.
Native recorded replay r2 covers20147physics ticks and80588wheel force values.
Maximum tire error1.90735e-6degree; maximum wheel drive error2.88142e-7m/s²;
sleep mismatches0, including1641sleep ticks and actual stop/wake transitions.
First tire state only initializes the queue; later measured tire values are not
used to update it. Initial0.3s unknown queue excluded from tire scoring. Full
single trace includes post-Finish; this is component parity, not race performance.

The replay uses private applied input and grounded masks. It proves those causal
component functions, not that future contact or DDS/application is available to
normal control. The remaining selection must join the correct base_link body
model, deployable current state, command application uncertainty and complete
Stop/Restart. Hard constraints, gains, safety margins and delays remain unchanged.

## Cached contact epoch and remaining body error

The inspected Wheel.UpdateWheelHit calls WheelCollider.GetGroundHit after the
current steering update; UpdateWheelForce uses that hit's cached direction and
point. Comparing current, previous and two-tick-old body/tire combinations on the
same measured ground plane establishes the previous physics epoch as the closer
direction source. In single16..59.54s, front direction MAE0.003670/0.004076rad
falls to0.00004180/0.00004301rad with previous body AND previous tire. Rear MAE
0.003455/0.003407 falls to0.00004651/0.00004242. Both dev2Domains improve too.
Residuals remain, including single maximum0.003388rad; this is not exact direction
parity. All0/1/2 combinations are retained in `contact-epoch-r1/report.json`.

`cached-contact-r1` uses past predicted body/tire and cached world contact points
with the original frozen contact weights, correct base_link, public initial state
and prescribed publications. It has no future contact/body inputs. The initial
previous frame is an explicitly estimated constant-twist backward projection.
Single0.5s position MAE changes0.056545->0.055794m;1s MAE0.193661->0.222850m and
maximum0.675902->0.692849m worsen. Therefore the contact-epoch correction is a
supported simulator component finding, **not a sufficient model repair or
promotion**. Do not fold5ms into the unrelated DDS/steering delay parameter.

Serialized `WheelCollider` extraction with read-only UnityPy1.25.3 shows all16
colliders have zero forward AND sideways friction curves/stiffness. Hidden
tangential WheelCollider friction is not supported. Suspension remains active:
spring35000, damper3500, targetPosition0.01, distance0.001m; radius0.12m, mass1kg.
Normal/contact/suspension reactions are not thereby removed from the body law.
These are local2025-derived asset values, not2026official constants.

## Application and Stop counterexample

`test_application_stop_boundary.cpp` composes the verified Drive component with
an explicit straight, all-wheel-grounded planar integration for an isolated
counterexample. It is not full rigidbody/road or actual-run stopping-distance
evidence. With the same25ms brake receive stream, a100mslatest-value selector at
phase0 selects the brake, while phase30ms overwrites it before selection. At0.2s
their modeled speeds differ1.255407 versus1.690412m/s. Packet loss is not needed.

With a continuously requested brake, the same simplified scenario stops at
0.327170m for immediate receipt versus1.436454m for0.5sdelayed receipt. A brake
not received during the2s window never executes. An unconstrained all-ungrounded
branch has no wheel braking force; its presence identifies an unclosed contact
abstraction, not proof that2sairborne motion is reachable on this road.
The native counterexample compiles with-Wall/-Wextra/-Werror and all assertions
pass. No existingMPCC test or physical tolerance was changed to obtain this.

The full completion plan's requirement for bounded application/model error is
still open. The user has been asked to distinguish empirical simulator acceptance
with explicit limits from a Stop certificate requiring guaranteed communication
and contact bounds. Neither missing reply nor an observed maximum supplies such
a bound. Production model adoption and dependent M2–M6 remain pending; supported
diagnostic components and rejected comparisons are saved independently.

Validation of this slice:72artifact/input hashes match; all new Python parses,
JSON reads, local documentation links and git whitespace checks pass. Central
registry validates140snapshots/282experiments. Strict native compilation and
Stop counterexample assertions pass. Original DLL and both protected userJSON
hashes match; no containers remain running. This does not add a new ROS package
build, simulator acceptance or completed model migration claim.
