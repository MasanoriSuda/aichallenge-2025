# Published Stop loss: observation and causal comparison

Baseline `f3b0338cb18b8ece3a494b7438bf35cac4dd35c1`. Full MPCC completion
remains open. The only production change in this slice removes
`published_stop_retained` from the authority-loss recorder's early-return guard.
The flag can select external Emergency after failed reproof; it is not a current
certificate. The existing bounded recorder now preserves the executed Stop before
Emergency clears the ledger. No command selection or runtime parameter changed.

## Verification and diagnostic run

The extracted native guard missed two Cruise/Track cases before repair; all six
cases pass after repair. `make autoware-build`: 25 packages pass. Container
`colcon test --packages-select multi_purpose_mpc_ros` and
`colcon test-result --verbose`: 60 CTest groups / 2346 records, zero errors,
failures or skips. The build emits the existing setuptools deprecation warning.

Run `20260909-published-stop-observation-dev2-r1` is an observation success and
an integrated rejection. Domain 2 first moving Emergency is decision 993,
observed speed 1.843041217111899 m/s, Ready, wall time 1788896720.209185064.
Shutdown starts at wall time 1788896725.1441615; both Domains stop in parallel.
No completed race or penalty acceptance is claimed. Both user JSON files are
restored byte-for-byte, with hashes in `artifact-restoration.json`.

The run seals the actual executed source 453, source fingerprint
8888292447036995152, interaction fingerprint 5828923344099263782, SHA-256
`f60754440e27bcd8a2df16606646f92648d318bf15f95715f3407a86554b1584`.
Failure world 993 has interaction fingerprint 8121216253231114284, SHA-256
`fb743a433474cce3a6aaed32dafb4072cb2a38ffa058349f8ffdbed383f6070c`.
Source observation / prediction origin: 11.529999742 / 11.659999742 s.
Publication decision 988 joins at control origin 11.759999740 with artifact
cursor 0.099999998 s. Failure observation / control origin:
11.744999737 / 11.874999737 s. The same bag records five moving braking
commands from 11.629999740 through 11.729999737, before Emergency.

## Zero-solve reconstruction

`replay_execution.py` reconstructs the actual execution evidence, including all
seven semantic initial states and the immutable source course frame. It never
solves a replacement QP. Original Stop 453 passes native wall, dynamic and
certified-plan checks; terminal speed is 4.50e-15 m/s against the existing
0.0046784965 m/s rest tolerance. Minimum peer clearance is 0.75156926 m.
Independent Cartesian DOP853 integration of the same acceleration/steering-rate
controls agrees across 484 samples: maximum position difference 0.003488 mm,
yaw difference 6.192e-6 rad. Virtual progress never supplies physical velocity.

Native revalidation against world 993 reproduces the runtime result:
delay and publisher interval clear; continuation wall blocked; both terminal
references fail. Original expected speed is 1.06705227 m/s, current observed
speed 1.84304122 m/s, predicted control-origin speed 1.91373332 m/s.
The separate Stop-successor path rejects `course-frame-unavailable`: physical
progress 32.624681 m is below its first virtual reference knot 32.783021 m.
A valid lag-coordinate representation exists, but this is not the primary
retained continuation failure and has not been repaired here.

Replay limitations: current physical steering is rounded to six decimals from
the log; publication age is reconstructed from exact consecutive command
stamps. A 1000 m path length substitutes only for an unused wrap length in this
local non-seam comparison; no lap association claim. The remaining world,
artifact, command-origin steering and response are sealed native inputs.

## Longitudinal prediction producer

`mpc_controller_cpp.cpp` calls
`predict_accelerating_yaw_response_trajectory` with the current observed
acceleration held over the entire 0.13 s prediction interval. Already published
acceleration changes are not passed to this producer. At decision 993, observed
acceleration is still +0.543785 m/s², although braking has been published for
0.115 s. Reconstructed current predictor matches the recorded terminal pose
within 1.14e-9 m, speed within 4.45e-16 m/s and response within 6.84e-8 rad.

Same-run velocity reports bracket the control origin at 11.865 / 11.900 s;
linear interpolation gives 1.45430135 m/s. Replacing only predicted velocity
with this future measurement makes terminal proof pass. This is hindsight
sensitivity, never an admissible production input.

A separate causal counterfactual uses only commands published before decision
993, applied at their already declared observation + 0.13 s origins. It retains
the current pose/speed/yaw observation, steering predictor, wall, peers and proof
requirements. It predicts 1.52263164 m/s and recomputes the complete measured-to-
control path. Native current-world terminal proof passes on its first reference.
No new normal command is produced. This proves the missing committed-input
producer changes the verdict on this sealed case, not integrated acceptance or
an exact identification of simulator application times.

Read-only local `Assembly-CSharp.dll` IL confirms VehicleRosInput takes the
latest queued longitudinal command at intervals determined by
`actuationLongitudinalUpdateRateHz`; the sealed YAML sets 10 Hz. Its phase is
not the controller's nominal 0.13 s origin. Keep that modeling limitation
explicit; do not tune delay to this single event. Sensor filtering and command
application remain distinct clocks.

## Architecture comparison and next repair boundary

On unchanged world 993, A, right B/C/D/G and all four fresh Stop arms solve but
fail the exact wall proof; left B/C/D/G fail QP constraints. C/D each search 210
bounded candidates, D uses its existing depth 3. All outcomes are retained under
`output/20260909-published-stop-architectures/`. The CLI header's `accepted=1`
means input accepted; every arm has `bundle=0`. No physical infeasibility
certificate exists; the scene remains Unknown under those hypotheses.

Next structural slice: bind latency prediction to successfully serialized
longitudinal inputs and their existing declared control origins. Establish a
native failing temporal regression first, define duplicate/backward/reset and
missing-history behavior, keep one shared pose/speed/response path and version
its context if necessary. Remove the obsolete constant-acceleration production
path in the authorized scope. Full build/package tests, sealed replay, and a new
single/dev2 acceptance trial are required before accepting that repair. Audit
course-frame successor projection independently; do not add retry/replenishment
rules or weaken wall/peer checks. Rollback for this observation slice: f3b0338c.
