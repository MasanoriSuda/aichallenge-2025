# Design: retained recursive Stop invariant

`mpcc_rate_resolved_retained_revalidation::evaluate()` already reconstructs a
fresh nonlinear continuation from the current physical control origin.  It
also has one exact terminal Stop proof path, but currently calls it only when
the continuation is partially clear.

Move that existing Stop construction and certificate to the common path for
both `FullSuffix` and `PublisherIntervalPrefix` results.  The Stop remains:

1. the already-selected serialized normal command for one publication period;
2. maximum braking with the declared path-tracking Stop policy;
3. exact nonlinear state integration;
4. exact static-wall, timed dynamic-obstacle and Follow-gap proof.

No alternate Stop policy or relaxed geometry is introduced.  This is a
contract repair: a full normal suffix no longer substitutes for recursive
stoppability, because its terminal state is not itself a certified Stop and a
later current-world rebase may invalidate the suffix before the next worker
result arrives.

The previous `partial_normal_proof` condition is removed rather than retained
as a second authority branch.  Tests assert that both proof scopes carry the
same terminal certificate.
