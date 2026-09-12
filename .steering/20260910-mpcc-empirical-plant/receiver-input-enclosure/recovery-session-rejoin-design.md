# Ready Recovery and certified Rejoin boundary

2026-09-13. Baseline3e674523; numerical productionfdee0b8f. M4 slice under the
existing autonomous implementation/local-commit authorization. No push.

## Evidence and scope

Single-r11Ready1048/source651loses the original certified Stop at rest expiry.
Both fresh initializers804/805are rejected by the exact full-footprint wall proof;
R653identifies front-left occupiedcell476325. Source geometry repair remains open.
R654/R655compare a target-free native nonlinear feasibility planner separately.
Local optimizer failure does not establish forward infeasibility.

A separate upstream supervision mismatch is deterministic: normal planning is
active during prepared Ready rollout, but DetectorInput.race_started receives
false untilStart. The detector rejects before collecting a stopped observation.
The original control-mode design explicitly preserves Recovery after motion or
engagement timeout (20260728-awsim-control-mode-reassert/requirements.md:24–26).
Current simulation-only, launch-engagement, control, gear, fresh odometry/V2X,
footprint/contact, incident/distance and boost guards remain mandatory.

Before enabling Ready Recovery, remove its old LowSpeedRejoin direct velocity,
proportional acceleration and steering synthesis. Rejoin is a normal intent under
the package AGENTS contract. The current arbitration instead marks it as a Recovery
override, bypassing immutable normal actuation matching and final normal dispatch.
This is a distinct authority defect, not evidence that an offline native candidate
may execute or that the stopped source's wall failure is repaired.

## Intended change

1. Represent Recovery eligibility as an operating session. Preserve existing Race
   behavior; add only tracked simulation Ready with a prepared autonomous rollout.
   Unprepared Ready, Spawned/Grounded/Finish and non-simulation prestart stay inactive.
   Feed the same meaning to detector, supervisor and fault-retry gate. Keep true
   reset boundaries and the existing Start reset/stop-confirm/Drive-report procedure.
   No new retry window, budget, margin or timeout. Existing launch engagement guard
   remains responsible for suppressing initial stuck detection.
2. Request Rejoin from the canonical nine-state producer while the supervisor is
   in LowSpeedRejoin. Apply its existing speed limit to the optimization before
   solution/certificate creation. Preserve deliberate Hold/Stop and complete peer
   and wall proofs. Intent changes invalidate pending/warm-start context normally.
3. Arbitration may release only the unchanged current certified Rejoin command
   within that cap, with Rejoin publication role. The normal dispatcher, ledger,
   source/certificate identity and actual publication deadline remain in charge.
   Pending/failed/other-intent/Stop-contingency results use bounded Emergency Stop.
   Remove raw rejoin target speed, 100x acceleration, steering replacement and the
   special non-normal rejoin model update. No second normal authority remains.

## Verification and rollback

Before production edits, preserve a failing source deletion regression for raw
Rejoin actuation and deterministic inactive Ready detector evidence. Native tests
must cover session eligibility and unchanged negative gates; Rejoin command tests
must reject missing/nonfinite/wrong-formulation/intent/identity/over-cap inputs and
accept exact valid fresh/retained commands without modifying them. Check source
wiring for cap-before-solve, distinct publication role, and no direct rejoin owner.
Run focused native tests, full26package build, package/source tests, relevant old
unsafe replay classifications, then a fixed-source simulator run with full Recovery
and publication telemetry. Separate local proof acceptance from actual restart,
Start/laps and integrated M4–M6acceptance. Revisit supported causes on failure.

Rollback baseline3e674523for this supervision/authority slice only. Preserve earlier
numerical work and all user artifacts. This design is not a report of implementation
or successful validation; those results must be recorded after execution.

## Local validation

R656:79execution-contract/149Recovery tests pass. The corrected source deletion
gates fail3on baseline3e674523and pass with all115source/architecture tests after
implementation. Build127all26packages and package114all2629records pass; see sealed evidence for
exact result totals and source hashes. Rejoin waits preserve candidate generation,
and a current certified Stop may retain its own Stop publication role. The nominal
vehicle kernel, wall/dynamic/full-rest proof, input uncertainty and deadlines are
unchanged. Ready-to-Start retains the original reset and confirmed-stop/Drive-report
procedure; no launch guard exception. Prospective single-r12is the next required
observation, not yet accepted.
