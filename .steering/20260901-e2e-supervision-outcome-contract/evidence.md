# Evidence

Generated report (not committed):
`output/20260901-e2e-supervision-outcome-audit.json`.

## Current contract audit

| Evidence class | Sequences | Samples | Material samples |
|---|---:|---:|---:|
| teacher: counterfactual on certified failure | 3 | 6,884 | 1,397 |
| teacher: successful alternative policy | 2 | 12,431 | 1,062 |
| teacher: outcome unproven | 10 | 34,393 | 6,033 |
| normal: successful alternative policy | 3 | 17,747 | 0 |

- Strict hard teacher demonstrations: `0`.
- Demonstrated zero-action normal sequences: `3`.
- Exclusive-action training contract: `fail`.

The audit maps recorded `/output/...` paths through an explicit host output
root, binds each result-detail to its ROS domain and fails closed on missing,
invalid or mismatched v3 evidence.  It also parses the executed
`tiny_lidar_control_mode` from the source log rather than inferring it from the
dataset name.

## Decision

Keep production v11 unchanged.  Do not train another candidate from the same
hard labels.  First collect and certify an exact teacher-controlled rollout;
otherwise redesign the positive label source around a policy that has actually
completed the relevant encounter.
