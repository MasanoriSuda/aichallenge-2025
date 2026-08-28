# Design

## Comparison boundary

The production path remains:

```text
async solve + exact proof
  -> candidate store
  -> later current-world adoption
  -> existing production authority or Emergency
```

Only when that path has no authority, the audit arm evaluates:

```text
current MpcProblem
  + exact previous serialized command
  -> same seven-state SQP
  -> same exact physical wall proof
  -> same current-world retained evaluator at cursor zero
  -> observation-only result
```

The audit plan is not stored, marked executed or published.  It uses a
dedicated solver context so it cannot race the asynchronous worker's warm
start state.

## Interpretation

- async rejected / synchronous accepted: publication scheduling defect;
- both steering-unreachable: formulation or predecessor binding defect;
- both solver-rejected: numerical/single-SQP limitation;
- solve accepted / exact proof rejected: model-certificate mismatch or real
  physical infeasibility.

The audit is temporary.  If it proves the scheduling defect, production is
restructured around a directly solved main MPCC and asynchronous work remains
only for tactical alternatives, matching the useful architectural property of
the upper-rank log.  The old async normal adoption branch is then removed in
the same migration slice.
