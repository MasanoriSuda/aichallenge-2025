# Design

The aggregate window owns two independent observations:

```text
last_result         = newest consumed result, regardless of outcome
last_failure_result = newest consumed result whose outcome is not Solved
```

Successful results may update `last_result` but cannot erase
`last_failure_result` inside the current reporting window. The aggregate reset
clears both after their details have been emitted.

The failure trace includes sequence, decision, intent, source/geometry
fingerprints, outcome, OSQP iteration/count/reset provenance and the existing
typed detail. No failure is retried and no command path reads this record.
