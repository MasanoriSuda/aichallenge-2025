# Follow canonical authority promotion audit

## Confirmed root cause

`evaluate_follow_retained_shadow()` constructs a certified canonical authority, extracts exact
actuation and reconstructs a world prediction. Unlike Track/Cruise, `FollowShadowCycleResult` has no
fields for the selected problem, solution, plan, cursor or prediction. The function records only
`retained_command_available=true`; the command itself is discarded.

`get_control()` invokes that evaluation for telemetry and then continues into the existing normal
solve. Therefore Follow has two conceptual producers but only the old path can publish. The root
correction is to preserve the already certified payload and terminate normal routing at the Follow
authority boundary.

## Rejected alternatives

- Enabling a migration flag while retaining both normal owners.
- Extending retained-plan age or weakening current-world target/wall proof.
- Tuning OSQP, speed, clearance or Follow distance before deleting the ownership ambiguity.
- Copying only speed/steering and losing problem/solution/cursor provenance.
- Falling back to legacy normal control while the Follow worker is pending.

## Implemented authority flow

`CanonicalNormalSelection` now carries the exact command, problem context, certified solution,
immutable plan, cursor and prediction selected by the current-world proof. Track/Cruise and Follow
share one publication adapter, including intent validation and final-command mutation detection.

The Follow routing boundary is explicit and exhaustive:

```text
non-Follow                    -> not owned by this boundary
Follow + complete selection  -> publish canonical normal authority
Follow + incomplete evidence -> canonical emergency stop
```

The Follow branch returns from `get_control()` at this boundary. It cannot reach low-speed handoff,
legacy three-state MPC or another fresh five-state normal solve. Recovery and emergency arbitration
remain allowed to override the resulting command after this boundary.

## Static evidence

- `FollowProductionNeverFallsThroughToAnotherNormalOwner` covers all three routing outcomes.
- The shared adapter rejects incomplete payloads and mismatched Follow/Track/Cruise intent.
- `make autoware-build` completed all 25 packages successfully.
- The workspace CTest run passed 39/39 tests.
- `git diff --check` reports no whitespace error outside the user-owned result artifact.

## Deterministic production replay

Replay artifact:

`output/20260823-202408-follow-production-replay/d1/autoware.log`

Observed production proof:

- Follow emitted `authority=production, selected=1` after current-world certification.
- Final output emitted `canonical-follow-retained-published`.
- The execution contract joined `intent=follow`, `formulation=velocity-progress-5state`,
  `authority=certified-normal-solution` with matching problem, solution, plan and decision identity.
- The worker completed 244/244 submitted jobs in the final observed Follow window, with zero
  exceptions, zero identity rejects, zero submission rejects and zero snapshot failures.
- The final steady Follow interval reported 39/39 accepted current-world selections. Worker snapshot
  and compute times were 0.158 ms and 1.336 ms in that report; result age was 0.015 s.
- No canonical normal command mutation was logged.
- The only two `legacy-mpc-solved` final traces were `intent=stop`; no Follow cycle published legacy
  normal authority.
- No callback overrun occurred during the Follow production interval. One later 26.032 ms overrun
  occurred during `LowSpeedAvoidance`, outside the promoted Follow path, and remains a separate
  Slice 5 runtime-quality observation.

The replay also exercised fail-closed cycles while the worker was initially pending and while live
stage-gap proof rejected retained candidates. Those cycles emitted canonical emergency authority;
they did not borrow a legacy normal command. This validates the ownership contract, not Follow
solver availability or race performance.

## Remaining concerns

- The current type and helper names still contain `Shadow` even though Follow now owns production.
  Renaming them is mechanical cleanup and is intentionally not mixed into the authority change.
- A close or stopped target can make fresh/retained Follow evidence unavailable for multiple cycles.
  That is now visible as fail-closed emergency behavior. Improving solver availability must be a
  separate root-cause Slice; it must not restore a legacy fallback.
- Hold has no legitimate zero-progress canonical producer yet. DynamicWait remains ShiftOut/Pass
  provenance and must not be relabeled Hold to satisfy migration bookkeeping.

## Conclusion

The Follow ownership defect is closed: one certified canonical payload reaches publication, and the
old normal Follow owner is unreachable in a Follow cycle. This Slice changes authority wiring only;
it does not tune parameters or claim Hold/Stop integration complete.
