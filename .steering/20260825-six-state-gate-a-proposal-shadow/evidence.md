# Evidence

## Root cause and implementation

The selected-side causal result used to be consumed in
`rate_resolved_normal_production_control()`. That function runs after
`init_problem()` has already called `update_overtake_line()`, so a complete
six-state result could never be present at the real FSM Gate A. This was an
ordering defect, not a solver or clearance parameter defect.

The consumer now runs after live `evaluate_v2x_behavior()` and before the
final `update_overtake_line()` call. Private async `init_problem()` clones use
`behavior_override != nullptr` and are explicitly barred from consuming the
shared mailbox. A proposal contains the exact selected Mission and its causal
six-state `CertifiedPlan`; target, side, prospective generation, tactical
sequence and context epoch must agree. The FSM deliberately ignores the
proposal in this observation Slice.

## Static verification

- `make autoware-build`: 25 packages passed.
- Full `multi_purpose_mpc_ros` package tests: 49 targets, 1,875 tests, zero
  failures, errors or skips.
- Source contracts prove live ordering, worker-consumer isolation, atomic
  Mission/plan transport and absence of store/publisher/FSM consumption.
- `git diff --check`: passed.

`colcon test-result` still reports the pre-existing missing
`build/joycon_contract_guard/package.xml` warning while the package test result
itself is clean.

## Dynamic verification

Bounded `make dev2` artifact: `output/20260825-223846`.

Domain 2 reached:

- 54 causal submissions and 38 consumed results;
- 37 complete solver/wall/target certificates;
- 12 current-world joins;
- 12 tactical-authority-ready results;
- 12 typed Gate-A proposals;
- successful proposals on both side signs (`-1` and `+1`).

One representative accepted result was sequence 28 with exact target `d1`,
side `+1`, prospective generation 1, result age 0.045 s, worker time
37.377 ms, solved six-state QP and accepted physical proof. A newer but no
longer reachable result was rejected as `steering-unreachable` and produced no
proposal, confirming fail-closed behavior.

Control callback telemetry remained below the 25 ms period. The largest
observed callback window maximum was 6.808 ms in domain 1 and 5.447 ms in
domain 2; every reported window had zero overrun. Startup odometry failsafe and
the bounded shutdown errors are outside this Slice.

## Verdict

PASS for shadow Gate-A observation. The six-state Mission and physical proof
can reach the actual live admission boundary atomically without changing
production behavior or callback scheduling.

Production promotion remains blocked until the authority Slice defines and
tests the intent coverage it will own. This run proves fresh ShiftOut entry;
it does not justify silently promoting an unobserved direct-Pass entry. The
promotion Slice must delete the corresponding five-state entry proof and
cache in the same change rather than retain them as fallback.
