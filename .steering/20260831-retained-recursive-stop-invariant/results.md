# Results: retained recursive Stop invariant

## Static verification

- The new blocked-Stop fixture failed before the implementation change.
- The focused retained-revalidation suite passed 60/60 after the change.
- The full package suite passed 2,287 tests with zero errors, failures or
  skips.
- `colcon test-result --verbose` still prints the pre-existing stale
  `joycon_contract_guard/package.xml` lookup warning; it is unrelated to this
  Slice and did not change the clean test summary.

## Dynamic Gate

Run: `output/20260831-022355` (`make dev2`)

- D1 and D2 both returned to certified normal Cruise authority and continued
  moving after transient Stop authority.
- D2 completed one full
  `Idle -> ShiftOut -> Pass -> Return -> Idle` episode (episode 1, waypoint
  IDs 37--62).
- That episode had zero `-> Recovery` transitions and zero
  `actual footprint wall margin violated` reports.
- The latest retained-revalidation aggregation windows were 80/80 accepted
  for both domains.  By construction and focused test, every accepted result
  now contains a certified current-world terminal Stop, regardless of
  `FullSuffix` or `PublisherIntervalPrefix` scope.
- D1 recorded two retained reason transitions to
  `terminal-contingency-unavailable`.  One example rejected a Cruise command
  whose normal publisher interval was clear but whose exact Stop rollout had
  `invalid-lateral-bounds`.  The vehicle later rejoined certified Cruise; no
  authority exception was added.
- Callback windows were normally below 25 ms, but a small number of overrun
  windows remain.  Timing-tail work is outside this invariant repair.

## Conclusion

The old failure allowed a wall-clear full suffix to advance without proving
that the next causal action could stop.  The repaired contract rejects that
state before publication and still permits a complete Overtake episode in the
dynamic Gate.  This closes the recursive-stoppability defect; it does not
claim six-lap race acceptance.

The remaining `terminal-contingency-unavailable` events are now explicit
candidate/Stop-contract evidence rather than a reason to restore the old
unchecked full-suffix path.  They should be handled upstream by generating a
recursively stoppable candidate, not by adding a lease, grace period,
clearance change or normal fallback.
