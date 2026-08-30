# Design

Add observation-only lifecycle logging at the two currently invisible causal
boundaries:

1. A current-world Gate A draft is built from the live Overtake selection.
2. The command for the same decision is serialized and the draft is either
   admitted to the worker or rejected with one explicit reason.

The existing worker-result log remains the third boundary, and the existing
entry-commit log remains the fourth boundary.  Together they form one trace:

```text
draft-built
  -> publication-admitted
  -> worker-result / Gate-A-proposal
  -> entry-commit
```

No retained trajectory or new tactical state is introduced.  No control value
or decision is changed.

