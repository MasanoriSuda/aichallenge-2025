# Design

Extend `CanonicalNormalCandidate` with two independent identities:

```text
execution_plan_id
execution_certificate_decision_id
```

`problem.decision_id` answers “which optimization produced this solution?”.
`execution_certificate_decision_id` answers “which current observation proved the remaining plan
reachable?”. They are equal for a fresh plan and intentionally differ for a retained plan.

```text
fresh:
  problem.decision_id = current decision
  execution_certificate_decision_id = current decision

retained:
  problem.decision_id = older solve decision
  execution_certificate_decision_id = current decision
```

The selector never manufactures either value. A future runtime store must advance the real control
sequence and run current-pose wall/obstacle revalidation before submitting the retained candidate.
