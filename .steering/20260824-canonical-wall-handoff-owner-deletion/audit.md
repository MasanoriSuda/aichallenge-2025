# Audit

## Observed phenomenon

The Overtake entry itself is accepted atomically, but its first command is
overridden after returning from the MPC controller:

```text
Overtake entry commit accepted ... required_wall=0.400 m
canonical retained proof ... reserve=0.253 m
Wall path admission ... scope=dynamic-escape-exit
  planner_contract=1/admitted=1/reserve=1.57 m
  execution_contract_mismatch=1
  path_wall=rear/0.11 m
Overtake control decision ... acceleration=-3.00 m/s2
  output=dynamic-escape-exit-wall-hold
  authority=legacy-normal-bypass
```

Roughly 0.47 seconds later the frozen DP source reaches its 0.5 second age
limit. The plant has already been decelerated below the certified plan, so the
time-indexed retained window and measured course progress diverge. That later
appears as `current origin rejected: discontinuous` and canonical Emergency.

## Data/control flow

```text
current-world-certified five-state Overtake plan
  -> canonical command returned by MPC::get_control()
  -> node retains stale DynamicEscape exit/wall gate state
  -> old outgoing prediction is independently reinterpreted
  -> retained/last steering hold replaces canonical command
  -> -3.0 m/s2 is published as legacy-normal-bypass
  -> plant falls behind the immutable time-indexed plan
  -> source expiry + retained progress rejection
```

## Root cause

The publisher boundary still has two normal-control owners. Canonical Overtake
owns a complete, physically certified command, but legacy wall-handoff and
DynamicEscape exit state remain eligible to restore another trajectory or
hold command after selection. The stale DynamicEscape gate is not scoped to
the execution lineage which activated it; in the observed run it had already
accumulated more than one thousand hold cycles.

## Why the later progress rejection is a symptom

`MpcProblem::progress_origin_m` is current measured `model->s`, while the
retained execution cursor advances the immutable plan by elapsed solve time.
Rejecting a large difference is correct fail-closed behaviour. Increasing the
1.5 m tolerance or changing the cursor would hide the upstream command
mutation and could replay a time-indexed control at the wrong vehicle state.

## Existing patch interaction

The active-Overtake legacy wall monitor already excludes
`canonical_normal_command`, and a source-contract test protects that rule.
The solver handoff, DynamicEscape wall monitor, retained DynamicEscape restore
and exit-lateral hold do not share the same authority exclusion. The partial
deletion therefore leaves a side door around the canonical production
boundary.
