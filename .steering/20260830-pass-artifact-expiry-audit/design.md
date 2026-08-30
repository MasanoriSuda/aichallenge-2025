# Design: Pass artifact-expiry audit

The opposite branch at decision 5683 is not a valid immediate rescue: the
vehicle is committed to the positive side and a full-track crossing is not
certified. The audit therefore asks why no new positive artifact replaces the
expiring plan.

The existing `mpcc_architecture_compare` executable rebuilds immutable
snapshots without any production publisher connection. Its arms distinguish:

- persistent/current pipeline failure;
- stateless current-world candidate success;
- rough lattice/polynomial candidate success;
- bounded offline multi-SQP success;
- common physical-proof rejection.

Only after this classification may the next production Slice replace one
owner. A solver threshold or an expiry grace would conceal the failure and is
out of scope.
