# Rejoin source wall and nonlinear velocity audit

2026-09-13. Authorized autonomous M1–M6 continues; local commits, no push.
Production remains9c522a95, build132/package119. No control/model/budget change.

## Live result and evidence scope

single-r18:4234callbacks,max19.482284ms;357normal sends,last901/job898/source512.
No before-send refusal or actual post-send window violation. Ready only,noStart/laps.
Recovery repeatedly enters LowSpeedRejoin, but no certified Rejoin publication or
completion occurs. The new post-Rejoin reserved recorder never becomes eligible;
its live capture is unverified. r17 is a different world and retains its two actual
Rejoin completions and later reserved-source timing problem. Runtime stopped and
original DLL/user JSON hashes restored. R706/R707retain same-run logs/public topics.

R708joins source527/537/882 to actual post-motion final loss912 and earlier actual
normal send901. Source sequences and control decision IDs are distinct. R710527
reproduces maximum-iteration rejection, not exact residuals. Source882 is version2
exact-QP evidence, not a complete interaction snapshot; the loader abort is preserved
as a setup/applicability failure. It is not evidence of physical infeasibility.

## Exact occupied-cell rejection

source537/control1104,clock16.284999636 has complete source/world/native input.
R711reproduces outerwall rejection at dense stage144,time0.725s,cell471815.
Initial/control origin is clear; nearest cell473318 has9.168mm separation.
Rejected cell471815 overlaps by0.520mm after about0.169m body displacement.
Logged distance2.886m is nominal reference path distance, not vehicle displacement.
Resolved footprint is front1.615,rear0.51,left/right0.968,margin0.05m,clearance once.

The telemetry physical_wall=accepted describes corridor/refinement construction;
it is not the final swept occupancy-grid certificate. Existing outer rejection is
correctly retained. Earlier A/H terminal rejection and stateless unsupported-intent
outcomes do not show an impossible Rejoin: audit terminal resolution omits target-free
Rejoin although canonical_normal_intent_requires_target(Rejoin) is false.

## Fixed structural comparison

R712changes only a copied diagnostic comparison source to admit empty-target Rejoin
under existing dynamic/braking guards. Original native inputs still reject atstage144.
Four predeclared native witnesses steer at rest to the original cumulative bound,
then drive2or4original stages and brake; two signs,virtual speed0,no safety changes.
Right-sign witnesses pass old wall/dynamic/Stop proofs. They are raw controls, not
normal authority. R714exposes that R712used a zero-valued default tolerance object
instead of SolverContext's tightened input bounds. R713also preserves a YAML C++
iterator compilation failure. R715rebuilds the same library with the original solver
owner: preparation now takes2stages; right2/right4again pass, leftsigns hit the wall.
The virtual progress coordinate is fixed, so motion is measured from lag/lateral in
the unchanged course frame, not from virtual progress. See exact values in evidence.

R716uses each accepted R715witness only as a nonlinear tangent, leaving all controls,
state/input bounds, references, costs and solver budgets free and unchanged.
Right4fails the wall-refined QP. Right2passes the old complete audit chain in82.78ms,
but closer inspection finds exact final speed2.317426407m/s versus the semantic2m/s
cap; the QP final speed is0.008201322m/s. This is NOT accepted normal authority.
The source physical adapter checks pose/lateral/nonnegative velocity and declared
terminal rest, but carries no original stage upper-velocity bounds. The audit result
therefore overstates complete semantic feasibility for this candidate.

R717compares the same right2seed with three predeclared diagnostic variants:

- A observes the old acceptance; first dense speed violation is sample772.
- B detects original0..2m/s violation using the unchanged solver row tolerance and
  uses the existing correction. Speed passes but original wall rejects atstage529.
- C additionally builds post-refinement tangents from a coherent exact native rollout
  of the solved inputs. One existing correction passes speed and full wall/dynamic/
  terminalStop proofs in90.28ms. Stage-end maximum speed1.293681m/s; final exact
  speed1.023594m/s. Raw dense and stage records are preserved; no command is sent.

C is evidence for a structural producer correction, not permission to bypass another
constraint. Its numerical state-selection helper still uses original bounds when
selecting tangents. Native transitions exactly match the dense physical endpoint
velocities in this case. No solver iterations, tolerances or margins were enlarged.

## Implementation boundary and remaining work

First repair target-free Rejoin diagnostic applicability with a failing regression.
Before promoting C, define and carry original semantic speed bounds through source
proof and current/retained programmes, including varying stage limits and Stop/rest;
verify other state constraints and fixed candidate generation on additional worlds.
Use the existing correction budget, no fresh retry/hold or safety relaxation.
Stationary steering preparation is an initial guess; all normal commands still require
an independently solved canonical nine-state trajectory and current publication proof.

The first post-Rejoin reserved worker input is still absent; R709was not executed.
r80D2actual deadline, measurement/contact model, Start/update_vmax, complete M4–M6,
repeated same-source races, multi-vehicle gates and the same submission/image/eval
remain open. [Evidence](rejoin-wall-source-evidence.json),
[prior post-Rejoin issue](post-rejoin-source-audit.md).
