# Requirements

## Objective

Classify the frozen Follow dynamic-obstacle failure from
`output/20260828-174825` without changing production authority or tuning any
constraint. Determine whether the physically late stay-behind branch is a true
dead end or whether a stateless current-world lateral escape can produce a
certified seven-state artifact.

## Constraints

- Keep the production Follow and Overtake authority paths unchanged.
- Reuse the same immutable interaction snapshot, seven-state SQP, wall proof,
  dynamic proof, solver policy and physical geometry for all arms.
- Do not change margins, tolerances, iteration limits, leases, timeouts or
  fallback policy.
- The Follow escape producer is audit-only and has no publisher/store/mailbox
  API.
- Do not enumerate Overtake-only C--G arms for a Follow snapshot.

## Exit evidence

- A fails and a certified B side succeeds: normal-intent tactical/candidate
  ownership defect.
- A and both B sides fail: retain the snapshot for a bounded C/D follow-up;
  do not claim physical infeasibility.
- A/B solve but proof fails: model/certificate mismatch.
