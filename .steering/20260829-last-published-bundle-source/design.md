# Design: last published Bundle source

## Root cause

Current-world Bundle publication correctly avoids calling `mark_executed()`
on an unmodified source plan. That fixed false provenance, but publication then
forgets the only source whose command actually crossed the wire. On the next
25 ms cycle, a Gate A source may not exist in the normal candidate Store, and
the asynchronous worker has not necessarily completed. Selection falls back
to an older intent, fails identity, and emits Emergency.

The defect is not that a Bundle needs a lease. The missing state is explicitly
allowed by the architecture: the last actually published certified artifact.

## Repair

Extend the certified-plan Store with a separate last-published Bundle-source
ledger. It records:

- immutable source `CertifiedPlan`;
- publication decision identity;
- publication control origin;
- artifact-local command cursor used by the proved Bundle.

It does not record the source as an exact executed plan. Retained evaluation
tries a newer candidate first, the last-published Bundle source second, and an
older exact executed plan last, deduplicating identical identities. The Bundle
source is consumed with a publication clock and passes the complete existing
current-world proof on every cycle.

When an exact plan command joins the publisher, the Bundle ledger is cleared
because the exact executed Store entry is now the last published source. A
failed serialization join or failed proof changes neither ledger.

## A/B/C/D classification

- A: current persistent-Mission / exact Store loses the Gate A Bundle source.
- B: stateless current-world Bundle with last-published source should bridge
  the worker interval.
- C/D are unnecessary unless B fails the unchanged proof.

Expected classification is a publication lifecycle defect.
