# Follow asynchronous canonical producer audit

## Confirmed observations

1. `get_control()` calls `evaluate_follow_shadow()` synchronously before it enters the existing
   normal/recovery solving branch.
2. `evaluate_follow_shadow()` builds, solves, validates, canonicalizes, stores and revalidates in the
   same callback.
3. The dynamic retained gate accepted 5 of its first 8 fresh-miss attempts and later accepted one
   additional command, so current-world execution does not require a same-cycle fresh solve.
4. The same run contains Follow solves at 4000 iterations and 20--27 ms. Around decision 4426 the
   enclosing callback reached 50.746 ms and 11/36 cycles exceeded the 25 ms period.
5. `LatestOnlyWorker` already supplies one-running/one-replaceable-pending semantics, exception
   containment, result sequencing and context-epoch rejection for tactical work.
6. `CanonicalExecutionPlanStore` is already thread-safe and rejects stale plan IDs.

## Cause chain

```text
Follow shadow eligible
-> five-state Follow QP solved synchronously in get_control
-> result remains shadow-only
-> get_control continues into legacy/recovery production solve
-> both computations consume one 25 ms callback budget
-> hard Follow QP reaches maximum iterations
-> callback deadline overruns although retained current-world execution exists
```

The expired retained plan seen later is a correct fail-closed outcome after a long non-Follow or
fresh-unavailable interval. Extending its certificate by age would be unsafe. The architectural
problem is requiring a fresh solve in the command thread, not the exact expiry reason.

## Hypothesis to validate

An immutable latest-only Follow producer plus current-world retained execution will preserve
canonical semantics while removing the double-solve deadline cost. This must be falsified if
snapshot construction itself, worker contention or result revalidation creates equivalent overruns.

## Rejected alternatives

- OSQP iteration/tolerance tuning before ownership separation.
- Sampling the synchronous shadow at an arbitrary lower frequency.
- Executing the last command by age without current-world proof.
- Publishing worker actuation directly from its old observation epoch.
- Adding another legacy/scalar fallback when the worker has no result.

## Slice A verification

The Follow solver context, warm-start identity/epoch and canonical plan store now share one
`FollowCanonicalLifecycle`. Short-lived tactical snapshots reference the same lifecycle rather than
constructing divergent state. Runtime solving remains synchronous in this sub-slice, so the change
isolates ownership without changing command semantics.

- `make autoware-build`: 25 packages successful.
- `ctest --test-dir /aichallenge/build/multi_purpose_mpc_ros --output-on-failure`: 38/38 passed.
- No normal authority, fallback, feature flag or parameter was added.

## Slice B verification

`follow_canonical_async` now defines a typed worker result and mailbox independent from the ROS node.
It rejects invalid plan payloads, internal plan/provenance mismatches, old context epochs, sequence
rollback and results that were never submitted. Rejected publication leaves the previously published
result unchanged; the mailbox cannot mutate the canonical execution-plan store.

- Formal `make autoware-build`: 25 packages successful.
- Focused CTest from `/aichallenge/workspace/build`: 3/3 passed.
- Full package CTest from `/aichallenge/workspace/build`: 39/39 passed.
- The older `/aichallenge/build` tree is not accepted as current evidence.

## Slice C static verification

Fresh Follow solving now occurs only inside the dedicated latest-only worker. `get_control()` submits
an immutable model/reference/problem snapshot, consumes a typed result, and runs the existing
current-world retained proof before a plan may replace the shared store. A worker plan is never used
as direct actuation. Leaving Follow increments the context epoch, so an in-flight old result cannot
publish into the new intent/target context.

The live identity gate deliberately accepts a newer target observation generation for revalidation,
but rejects context epoch changes, intent/target changes and observation rollback. Those semantics
are deterministic tests, not an age lease.

- Formal `make autoware-build`: 25 packages successful.
- Full current workspace CTest: 39/39 passed.
- `evaluate_follow_fresh_shadow()` has no live call site; its only call is the worker lambda.
- No controller parameter, normal authority, legacy fallback or direct worker command was added.
- Rollback boundary before worker connection: `fbd8f8f`.

## Remaining dynamic falsification

The callback still constructs an immutable model/reference snapshot, and the worker competes for CPU
with other tactical work. The dynamic gate must therefore measure snapshot cost, callback tail,
worker replacement/publish reasons, result age and current-world acceptance. Static success is not
evidence that the 25 ms runtime deadline is repaired.

## Dynamic gate attempt: 20260823-191318

The normal two-vehicle run routes the 15 km/h vehicle through LowSpeedAvoidance or the all-vehicle
dynamic-obstacle Cruise authority before a coherent Follow contract exists. A diagnostic-only run
therefore disabled those two owners and OvertakeLine without committing the configuration change.
Domain 1 then entered a coherent Follow window against `d2` at about 4--6 m.

- Worker: 268 submitted, 268 started, 268 completed, 0 exceptions.
- Callback: no 25 ms overruns in the observed Follow window; logged maxima remained below 15 ms.
- Snapshot construction: about 0.17--0.39 ms in the observed window.
- Mailbox: 0 accepted, 268 invalid; no result reached current-world proof.

This falsifies Slice C as production-ready. The asynchronous scheduling boundary works, but every
completed payload is rejected before publication. The existing mailbox telemetry collapses all
typed validation failures into `invalid-result`, so the immediate next action is observability at
that contract boundary, not solver or clearance tuning. `MailboxState` now exposes the exact last
`ResultValidationReason`; the diagnostic run must be repeated before any root-cause correction.

The repeated diagnostic run (`20260823-192406`) classified every rejection as
`invalid-identity`. Static data-flow tracing then isolated the mismatch:

```text
Follow has no overtake Mission
-> authority trace mission_generation = 0 (valid canonical meaning)
-> make_problem_context seals a complete Follow context with intent_generation = 0
-> async ResultIdentity copies that complete context
-> validate_worker_result independently requires intent_generation > 0
-> every completed worker payload is rejected before solve outcome consumption
```

`mpcc_execution_contract::problem_context_complete()` intentionally permits generation zero;
leaving Follow still changes the async context epoch, and target identity plus observation generation
remain mandatory. The root correction therefore removes only the contradictory nonzero check from
the mailbox validator and adds a regression test for a complete Follow plan with generation zero.
No timing lease, fallback or parameter exception is introduced.

## Dynamic gate attempt: 20260823-192924

Repeating the same diagnostic configuration after accepting missionless Follow identity proved that
the async scheduling and mailbox boundary are operational:

- More than 500 jobs were submitted and completed in the observed Follow window.
- Mailbox publication had no invalid payloads; completed results were consumed by the live thread.
- Snapshot construction remained about 0.17--0.27 ms and result age about 0.025--0.035 s.
- Worker exceptions and 25 ms callback overruns remained zero in the observed window.
- No worker plan reached `current_ready`; accepted mailbox payloads were typed rejections.

The repeated canonical rejection was `intent-mismatch`. Static data-flow tracing established the
upstream cause:

```text
live controller seals a complete Follow MpccProblemContext
-> ResultIdentity is derived from that context
-> tactical_snapshot copies the decision and physical problem inputs
-> worker evaluate_follow_fresh_shadow re-derives context from snapshot state
-> snapshot has no last_overtake_authority_trace_
-> current_control_intent() falls back to Cruise
-> canonical command adapter receives Cruise problem + required Follow intent
-> authority rejects the otherwise solved candidate as intent-mismatch
```

Copying the mutable authority trace into the snapshot would hide the ownership defect. The structural
correction instead passes the immutable, sealed live `MpccProblemContext` with the snapshotted problem
and prohibits the worker from re-deriving authority. A typed snapshot-context validator rejects an
incomplete, non-Follow or identity/fingerprint-mismatched job before solving. The evaluator also
checks that the sealed context matches the Follow target observation, horizon and stage geometry of
the physical problem.

- Formal `make autoware-build`: 25 packages successful.
- Full current workspace CTest: 39/39 passed.
- Regression tests cover exact sealed context acceptance, Cruise re-derivation rejection and a
  different sealed problem fingerprint.
- No authority promotion, fallback, lease, flag or parameter change was introduced.

This is still not a passed dynamic gate. A same-condition rerun must show a worker-produced canonical
plan reaching live current-world revalidation before Slice C can be closed.

## Deterministic dynamic gate: 20260823-200200-replay

Repeated live launches did not recreate a coherent front-target interval, and one three-domain
attempt remained in AWSIM `spawned` without a start-service subscriber. Those runs cannot falsify
the Follow producer. The accepted gate therefore replayed the exact observation stream from
`output/20260823-192924/d1/rosbag2_autoware` into a clean Domain 1 controller while excluding the
recorded `/control/command/control_cmd`. The replay supplied clock, localization, trajectory, V2X
and vehicle-status inputs; the current controller remained the only command producer.

The sealed-context correction passed the dynamic gate:

- Worker: 1405 submitted, 1405 started, 1404 completed, 0 exceptions and 0 snapshot failures.
- Mailbox publication: 1404 accepted, with 0 invalid/context/rollback/unsubmitted rejections.
- Live current-world proof: `current_ready=574`, repeatedly reporting
  `canonical-ready-worker` and `async-current-world-ready`.
- Snapshot construction remained about 0.13--0.25 ms in observed reports; normal worker compute was
  about 1--3 ms and result age about 0.015--0.035 s.
- Hard infeasible frames could still reach about 19--21 ms and maximum iterations, but that work
  remained off the command callback and produced typed rejection rather than direct actuation.
- Callback reports remained below the 25 ms period in the observed replay, with zero overruns and an
  isolated maximum of 15.493 ms.
- The two current-identity rejections occurred at Follow exit/re-entry boundaries. They rejected old
  async plans as designed; no `intent-mismatch` remained.

This closes the asynchronous Follow producer Slice. It proves current-world shadow availability and
latency isolation, not production authority. Follow remains `authority=shadow, selected=0` until a
separate promotion Slice connects this exact canonical selector to publication and deletes the
Follow-specific normal command owner in the same change.
