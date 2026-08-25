# Design

## Causal chain

```text
sealed six-state MpccProblemContext
  -> async solver Snapshot::Identity
  -> immutable ExecutionArtifact::Identity
  -> physical Snapshot / CertifiedPlan
  -> current-world retained Proof
  -> publisher-shaped command Candidate
```

The source context is immutable along this chain.  The retained proof's
`decision_id` remains the current execution-certificate decision and is not
substituted for the source context's original decision.

## Change

Replace the copied `decision_id`, `source_problem_fingerprint`,
`stage_geometry_id`, `intent` and `formulation` members in the rate-resolved
artifact identity with one `MpccProblemContext source_context`.

`identity_valid()` requires:

- `problem_context_complete(source_context)`;
- Track or Cruise intent;
- `VelocitySteeringProgress6State` formulation;
- a non-zero artifact sequence and a finite snapshot time.

All downstream comparisons and telemetry read through `source_context`.  No
context is rebuilt from a fingerprint and no current context is substituted
for a retained source.

## Alternatives rejected

- Reusing `CanonicalNormalCommand`: its builder and final decision contract are
  explicitly five-state and would hide the formulation change.
- Keeping both copied fields and the complete context: this creates two
  identity owners and permits disagreement.
- Connecting production first and filling trace fields later: that would allow
  an untraceable normal command to own the publisher.
