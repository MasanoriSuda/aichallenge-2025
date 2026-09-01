# Design

## Gate order

```text
promoted three-lap checkpoint
  -> six-lap practice (one ego + two runtime NPCs)
  -> four-vehicle final reference
```

The first gate extends encounter duration while keeping one student authority.
The second adds four independent ROS domains and peer interaction.  This order
prevents a multi-domain startup failure from being confused with a policy
generalization failure.

## Evidence

For each domain, run `analyze_e2e_run.py` against the finalized bag and retain:

- duration, forward distance and maximum speed;
- longest post-start low-speed interval;
- positive-acceleration stall interval;
- frontal LiDAR clearance and synchronized command/pose context.

Correlate that output with AWSIM `Finish`, collision/penalty strings and launch
logs.  The analyzer is a stall gate, not a lap or collision oracle.

## Change policy

The initial run is observation-only.  If it fails, classify the first causal
event and create a separate implementation slice.  Do not mix model retraining,
longitudinal threshold tuning and launch repair in one commit.
