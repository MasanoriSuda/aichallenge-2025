# Recovery confirmation and complete clearance

2026-09-13. Baseline9950c145, single-r12. Existing autonomous implementation and
local-commit authorization continues. No push. Integrated acceptance is incomplete.

## Observed failures

R659/R660:120sReady-only;4246callbacks, maximum36.770233ms at865;336normal
publications end864/source453. Before-publication guard865rejects9.904999778after
9.894999827deadline. No out-of-window normal send. Proof consumes33.926656ms wall,
22.603863ms CPU with20involuntary context switches; Recovery costs0.020ms in that
callback. Thus the first deadline failure is not caused by Recovery rollout cost.
No Start/laps/Reverse/Rejoin. Publicmaximumspeed0.994773388mps. Runtime stopped;
original simulator DLL and userJSON restored/preserved.

Ready905has a Confirmed stopped wall observation with a healthy normal solution.
Its HoldStop invalidates normal execution.906therefore observes solver fallback
and restarts the fallback qualification interval. SuspectStuck then returns to
Normal, losing the already accepted confirmation.887solver-unsafe/886awaiting/
885confirmed-hold events form the repeating loop; Store continues supplying fresh
certified sources (unlike single-r11). The producer defect is confirmation ownership,
not insufficient timeout or absence of a solver solution.

R661reproduces this loss. R662's direct Normal-to-Wait alternative changes the
existing stage sequence and fails57tests; rejected, original outputs retained.
R664keeps the original sequence and still reproduces the confirmation loss.
R665uses the existing StuckConfirmed state reason to carry the accepted fact into
the next wait, retaining all current supervisor safety guards and original wait/
gear timing. All150Recovery tests pass. No tolerance, retry, timer or actuation change.
Build/package/live validation of this uncommitted slice is still required.

## V2X producer and retired unsafe policy

R663reads2048fresh monotonic empty V2X arrays at20Hz. The system's existing
single_vehicle_empty_v2x_publisher is enabled only for a declared single-vehicle
simulation. The Recovery identity tracker requires a nonempty learned set, so it
reports all those arrays incomplete.1951runtime records instead use the legacy
incomplete_v2x_ignored override. Current code can also override self-identity and
actual peer blockage, select a rejected least-bad static primitive and reset retry
budgets on an unchanged scene. These are pre-existing force-motion policy paths;
they are not accepted hard clearance proofs and must be retired before moving
Recovery is treated as safe integrated behavior.

Share the declared scenario size with Recovery via an additive local launch/ROS
parameter recovery_vehicle_count. Source: existing AIC_VEHICLE_COUNT; default0
means unspecified, retaining conservative nonempty identity discovery. A declared
simulation count permits exactly N-1peer IDs with excluded-self arrays, or N with
explicit self identity. A declared zero-peer world still requires a fresh, valid,
current-Recovery-epoch empty message and an empty learned set; no message, stale
source/receipt, reset epoch, unexpected ID and multi-vehicle absence remain rejected.
Do not infer single-vehicle topology merely from receiving an empty array. Real
vehicle behavior cannot use this simulation topology declaration.

Retire aggressive_force_motion_enabled, rejected static candidate selection,
incomplete/self/blocker V2X overrides and unconditional unchanged-scene retry resets.
Keep the existing safe candidate generators, all physical/contact/peer proofs,
confirmed-stop/gear/Boost checks, bounded episode budgets and canonical Rejoin.
Diagnostic forced-candidate/override fields may remain false for log compatibility.
The old config key becomes obsolete and is removed from shipped YAML; it grants
no authority in an older external YAML. This is stricter local behavior, not a
2026official interface or simulator specification change.

## Verification and completion boundary

Record launch/consumer compatibility in participant-interface before wiring the
additive parameter. Native identity tests cover declared0/1/2/3peers, unknown
membership, duplicates/unexpected IDs and peer disappearance. Existing freshness,
reset, reverse/forward/gear and safe-stop negative tests remain required. Delete
only tests for the deliberately retired force-motion behavior; retain corresponding
strict blocked/unchanged-scene tests and add source deletion gates for bypasses.
Then focused native/source tests, full build128/package115, frozen-source review/
commit and bounded single-r13. The865deadline remains a separate pending exact
replay/cost investigation; never extend the25mswindow. AllM4–M6acceptance remains.
Rollback9950c145for this slice; preserve fdee0b8fnumerical work and user artifacts.

## Validation of the implemented slice

R667:148Recovery and825V2X native tests pass; source117pass. Build128finishes all26
packages with frozen inputs, package115: Summary: 2633 tests, 0 errors, 0 failures, 0 skipped. R669uses actual ROS launch
expansion for unspecified/single/multi/explicit override; all5cases pass as integers.
R668preserves the exact single-r12decision865input and its deadline for separate
replay/cost inspection; see evidence JSON. No normal numerical/proof parameter changed.
Next is source-fixed single-r13and its Recovery/gear/clearance/current-Rejoin audit.
Integrated Start, restart, laps, all-intent/multi-vehicle/submission acceptance remain open.
[Sealed evidence](recovery-confirmation-clearance-evidence.json).

R668reconstructs the exact source/domain fingerprints, actual current history and
last actual source. All3replays reproduce the captured current check and original
decision-clock pass / before-clock deadline refusal. Isolated dispatch takes
13.364–13.490ms with0involuntary switches, against live native proof33.927ms wall /
22.604ms CPU /20involuntary switches. The live scheduling contribution is observed;
the remaining CPU-cost difference is unexplained. This is not a25mslive guarantee.
