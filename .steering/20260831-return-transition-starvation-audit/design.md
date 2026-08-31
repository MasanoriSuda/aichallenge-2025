# Design: Return transition starvation audit

Keep the existing worker, mailbox and admission behavior unchanged.  Join the
following existing observations at their current boundaries:

1. Return transition requests a proposal but receives none.
2. The causal worker finishes a Return job.
3. The latest-only mailbox accepts or rejects that completed result.
4. The consumer evaluates current-world and tactical identity.

The worker completion log is bounded to Return jobs.  It records existing
immutable fields before the result is moved into the mailbox.  The deferral
log adds only generic worker statistics.

Classification:

- no completion while requests accumulate: scheduling/worker starvation;
- completion but mailbox rejects: latest-only lifecycle defect;
- mailbox publishes but consumer has no proposal: current-world or identity
  admission defect;
- consumer receives a solved but physically rejected result: candidate or
  physical feasibility defect.

