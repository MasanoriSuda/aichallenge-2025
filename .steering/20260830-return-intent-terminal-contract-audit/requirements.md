# Requirements

## Baseline and evidence

- Baseline: `831232dc refactor(mpcc): retain dual current-world normal branches`
- Dynamic run: `output/20260830-053105`, Domain 1/2
- Reproduced twice:
  - `Idle -> ShiftOut -> Pass -> Return -> Recovery`
  - Recovery reason: `actual footprint intersects static wall`
- First episode:
  - `Pass -> Return`: decision 2208, waypoint 143;
  - first joined Return bundle: decision 2228;
  - Return authority subsequently alternates with external Emergency Stop;
  - steering becomes unreachable and the delay path collides before decision
    2396;
  - physical wall contact is reported at decision 2406, waypoint 164.
- Frozen Return snapshots:
  - sequence 1592: persistent/current Mission side rejects while stateless
    opposite-side B accepts under the same SQP and physical proofs;
  - sequence 2384: every A--D family rejects, showing that the later state is
    already physically infeasible.

## Root cause

The high-level Return transition and the stateless MPCC candidate producer do
not share one semantic contract.

At decision 2208 the FSM selects a speed-preserving Return because the failed
Pass target is about 12 m ahead and physically clear. The Return preflight
builds a path from the current lateral position back toward the racing line.
The production stateless producer nevertheless treats `Return` exactly like
`ShiftOut` and `Pass`: it rewrites the lateral reference to the Mission
`execution_side_sign` and asks the dynamic-obstacle refinement for a
behind-to-pass-side topology. The live trace confirms 11 pass-side rows and 9
diagonal rows for a canonical `return` solve.

Consequently one canonical intent means two incompatible things:

- supervisor: rejoin the racing line behind the still-ahead target;
- candidate producer: start or continue a pass on the old Mission side.

The architecture comparison also lacks a required semantic certificate. Its
`TerminalSuccessor::Return` is resolved from the source request bounds before
solving and is copied into the accepted bundle. It does not prove that the
candidate's solved terminal state actually reaches the Return successor. The
stateless opposite-side acceptance is therefore physical feasibility evidence,
not yet a valid Return artifact.

The late Emergency and wall contact are downstream symptoms. By sequence 2384
all compared architectures are infeasible; changing wall clearance or solver
tolerance there cannot recover the earlier lost semantic branch.

## Objective

1. Give `Return` a candidate contract distinct from ShiftOut/Pass.
2. Rebuild Return candidates from one immutable current-world snapshot while
   preserving the Return-to-racing-line reference.
3. Select the dynamic-obstacle topology from current physical relation:
   - target ahead: remain longitudinally behind while rejoining;
   - target rear-clear: remain longitudinally ahead while rejoining;
   - current side overlap/separation: retain the physically occupied side
     until a certified rejoin successor exists.
4. Certify terminal Return viability from the solved execution artifact, not
   from source bounds or a Mission declaration.
5. Reject any physically safe candidate which does not satisfy its canonical
   intent before it reaches the Store or architecture-comparison bundle.
6. Remove the obsolete edge which applies pass-side reference rewriting to a
   Return candidate.

## Constraints

- Do not change production authority, control frequency, horizon, weights,
  solver tolerance, wall/vehicle clearance, acceleration, braking or steering
  limits.
- Do not add a lease, grace period, timeout, retry, resume rule, fallback or
  feature flag.
- ShiftOut and Pass topology behavior must remain unchanged.
- Return topology is derived from the same immutable ReplayWorld and exact
  vehicle geometry used by final proof; no persistent path sample or Mission
  corridor may authorize it.
- Target-ahead, target-rear and side-overlap relations must be explicit and
  mutually exclusive. Ambiguous geometry fails closed.
- Terminal semantic proof is additional to, never a replacement for, exact
  wall, dynamic-obstacle, reachability, actuation and Stop-contingency proof.
- External Emergency remains the only authority when no complete Return
  artifact is certified.
- Generated output, rosbag and architecture snapshots stay untracked.

## Acceptance

- Tests reproduce that the old Return builder rewrites a centre-return
  reference into a pass-side reference.
- Tests prove target-ahead Return uses a stay-behind topology and preserves the
  Return reference.
- Tests prove target-rear Return uses an ahead topology and preserves the
  Return reference.
- Tests prove a physically accepted opposite-side path is rejected when its
  solved terminal state does not satisfy Return viability.
- Architecture comparison reports semantic-terminal rejection separately from
  physical rejection.
- Full package CTest and `make autoware-build` pass.
- Dynamic trial completes `Pass -> Return -> Idle` without the reproduced
  pass-side Return rows, authority chatter, steering-unreachable cascade or
  wall contact.
