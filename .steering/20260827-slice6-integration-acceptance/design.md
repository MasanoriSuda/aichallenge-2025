# Design

## Authority under test

```text
observation snapshot
  -> tactical intent / target identity
  -> fresh seven-state solve or retained current-world proof
  -> exact executable stage prefix
  -> canonical normal authority
  -> serialization seal
  -> /control/command/control_cmd

Emergency / Recovery supervisor
  -> typed exceptional replacement only
```

The Gate audits this chain from the earliest input identity to the serialized
command. A downstream symptom is not repaired until the earliest producer that
violated the chain is known.

## Evidence collection

For each run, collect:

- result JSON: laps, penalties and abnormal tails;
- intent/phase transitions and target/generation identity;
- aggregate decision source, formulation, proof scope and rejection reason;
- fresh/retained selection and executable stage count;
- solver success/failure and branch result age;
- callback average/max/overrun counts;
- Emergency/Recovery initiator and preceding normal decision;
- command/odometry/V2X sequence from MCAP for ambiguous episodes.

## Failure classes

1. `InputIdentity`: stale/jumping V2X, pose or course projection.
2. `TacticalLifecycle`: wrong target/generation/phase transition.
3. `FreshProof`: current solve or complete-horizon certificate unavailable.
4. `RetainedProof`: current-world/current-stage continuation unavailable.
5. `PhysicalProof`: wall, dynamic obstacle, actuator or Follow hard contract.
6. `Authority`: valid proof exists but normal authority is not selected.
7. `Serialization`: certified command differs from published representation.
8. `Realtime`: worker/callback result is too late or starves the next cycle.
9. `ExternalPhysical`: AWSIM collision/penalty precedes controller reaction.

Only the earliest applicable class is the root class for a given episode.

## Repair rule

Every production change must map one demonstrated cause to one invariant. Do
not combine tuning or unrelated behavior changes. If a repair promotes an
authority path, remove the displaced normal path in the same Slice. If no
structural defect is demonstrated, record the Gate result without changing
production code.

## Structural repairs found by this audit

The integration audit found defects in the proof chain rather than a need for
new tactical tuning.

1. The sealed problem identity used the retired horizon length instead of the
   actual seven-state QP horizon. A valid current problem could therefore be
   compared with, warmed from, or retained under the wrong identity. The
   canonical context now seals the effective solver horizon, and snapshot and
   bound construction reject any mismatch.
2. A first solve without a compatible retained artifact did not have a warm
   start owned by the current affine dynamics. It now bootstraps state and
   input values from the current QP itself; the bootstrap is diagnostic and
   cannot be mistaken for retained execution evidence.
3. Wall and dynamic-obstacle refinements changed the optimized path after the
   temporal Frenet dynamics had been linearized around the semantic reference.
   Exact nonlinear replay consequently rejected millimetre-scale path/model
   disagreement. After those physical rows are finalized, one SQP correction
   relinearizes only the temporal dynamics around the refined primal and
   resolves the same costs and physical constraints. Failure at any step is a
   closed Gate, not a legacy fallback.
4. Wall bounds and dynamic-obstacle exclusions had overlapping ownership.
   They are now separate QP row classes with independent provenance and
   diagnostics. Dynamic partial escape is permitted only when the certified
   wall-only solution demonstrates a non-worsening lateral envelope.
5. Retained execution had several places where old predicted velocity,
   incomplete suffix coverage or a DynamicWait label could be treated as
   current authority. Revalidation now starts from current measured velocity,
   requires current-stage/full-prefix proof and preserves mission identity and
   lateral authority.

No wall margin, speed, lease, timeout or solver setting was changed to make a
failed proof pass.

## Static deletion boundary

The full package contract test asserts that the retired three-state and
five-state normal solvers, converters, stores, formulation switches and normal
fallthrough are not reconnectable. The normal publisher accepts only a
serialized command matching the canonical certified actuation. Emergency and
Recovery remain typed external replacements. Reference generation and tactical
FSM code may remain, but they cannot solve or publish an alternate normal
command.
