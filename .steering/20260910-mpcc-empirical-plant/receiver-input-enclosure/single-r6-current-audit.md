# Single-r6 current authority audit

2026-09-12 JST. Production8d9810e0, build120/package107. Standard single-r6
terminates on first moving Emergency968, not race completion. No Start/laps.
Original user JSON and simulator DLL restored; production remains unchanged.

R525 reproduces original source/domain/history/last actual publication and
six-component rejection vectors for both final active captures, three times:
784 is Track-to-Cruise context mismatch; later Cruise supply resumes. Final moving
968 retains source642/job961/index4 after actual967/index3, with valid source
context and identical geometry7297210173200458920. Current full wall proof rejects
at12.629999725. Nominal12.284999826/entry12.289999725/deadline12.309999826; before/
after zero means that publication bracket was unobserved, not a clock violation.
All1166 recorded callbacks below25ms, maximum21.074611ms;968 takes8.281163ms.
This failed short run does not resolve prior genuine clock failures.

R526 samples four input-bound endpoints using the unchanged native kernel and
original footprint+clearance. Both minimum-acceleration traces stay clear; maximum
acceleration traces contact original cell476325 at12.649999725/12.644999725.
These are admitted substep abstraction witnesses, not actual receiver traces,
not complete input enumeration, and not a physical infeasibility certificate.
They refute repairing this current rejection merely by tightening numerical boxes.

R527 retains the full original source tube. At fresh pose/velocity time
12.284999725, original speed range upper0.1789536995 versus observed0.2281698585;
relative yaw upper0.0013881290 versus observed0.0024820591. Position lies within
the original box. Original velocity timestamp12.074999730 differs from original
pose timestamp12.084999729; sensor skew and nominal plant error are separate
hypotheses. The current yaw-rate timestamp12.249999726 must not be compared as
if sampled at12.284999725. No model coefficient or uncertainty bound is changed.

The original solved source642 asks for negative steering rates near-0.714rad/s.
Its executed programme repeats semantic steering0.04450385 (wire0.06386302) for
three positive packets and terminal braking. The source programme generator at
mpcc_rate_resolved_retained_revalidation.cpp:1613 intentionally copies the first
angle for every future packet. The resulting steering responsiveness depends on
fresh source availability. This is a candidate-generation contributor hypothesis,
not yet a demonstrated cause of the model discrepancy or a certified repair.

Next bounded OFFLINE comparison on the ORIGINAL job961 source request:
original production versus fixed3/8/16 packet prefixes, same first packet,
acceleration and braking timing, with constant angle versus integrated original
solved steering rates. Both definitions fixed before evaluation; changed input
programmes get new input fingerprints. Reject rate/angle/horizon violations;
retain full native nominal+applied wall/peer/rest checks and independently verify
the immutable source velocity ceiling over the whole tube. Variable schedules
must not be labelled constant-steering. No current968 history is fabricated to
match a counterfactual programme. This comparison grants no production authority;
a positive result needs explicit source/identity integration and regression/live
validation. Existing A/B/C/D source comparisons and empirical plant limits remain
applicable. No new production fix or full M4–M6 acceptance is claimed.

## Comparison result and next observation

R528 original and constant3 reproduce input fingerprint8882791144668049159.
Source-rate3 also certifies with identical first packet and new input fingerprint
16201169140066102269; its constant-steering flag is correctly false. Both8-prefix
arms and rate16 reject the full wall proof; constant16 rejects nominal terminal.
The complete certified tube obeys the original velocity ceiling. No longer safe
programme or integrated repair is established. The proposed-arm field named
original_source_fingerprint_exact is unconditional in this diagnostic and is NOT
used as evidence; only baseline actually checks that field.

R529 bag trace includes later Emergency commands; those post-failure departures
cannot be attributed to the original programme. R530 explicitly ends comparison
at12.289999725. Public yaw leaves the original tube at12.124999728; speed and
lateral velocity at12.144999728, before the final failure. These are source-time
observations, not controller receipt/application acknowledgements. Maximum whole
bag speed1.1278783083mps; command source41.4696Hz includes26duplicates/startup/
teardown, no backwards/nonfinite values, maximum positive stamp gap35ms.

Next use the existing validated seven-call force/application probe on a bounded
single diagnostic. Original simulator CIL and generated DLL hashes match; no
controller or model change. Observe actual applied input, wheel forces and next
physics state to separate source-time skew, nominal force law and contact error.
This is a new observed world on8d9810e0, not a refit of the rejected old contact
model or performance acceptance. Keep all actual failures and final cleanup.
[Sealed evidence](single-r6-evidence.json). M4–M6 remain incomplete.
