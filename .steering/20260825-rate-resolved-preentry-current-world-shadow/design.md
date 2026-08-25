# Design

## Causal boundary

The prospective worker proves an immutable six-state trajectory against its
captured static wall and target tube. That proof is necessary but not sufficient
for adoption: async delay may change ego actuation, progress or dynamic obstacle
occupancy before the live controller commits the Mission.

The live boundary therefore performs:

```text
selected six-state CertifiedPlan
  + matching selected side
  + matching/fresh target provenance
  + current measured-to-control path
  + current wall identity
  + current dynamic obstacle observation
  -> existing rate-resolved retained revalidator
  -> observation-only accepted/rejected evidence
```

Static-world identity is content-based. The physical snapshot seals a
deterministic fingerprint over occupancy-grid geometry, axis convention and
cells. The revalidator keeps an O(1) fast path when the immutable grid owner is
shared and compares the fingerprint when an async boundary has produced an
equivalent deep copy. Pointer address is not world provenance.

## Authority invariant

The current five-state Gate A remains production Mission evidence in this
Slice. The six-state result is telemetry only and cannot influence
`apply_mpcc_entry_execution_contract`, FSM state, retained production storage or
the publisher.

## Promotion boundary

If dynamic evidence is adequate, the next atomic Slice may replace the
five-state pre-entry plan with this selected and current-world-revalidated
six-state CertifiedPlan. That promotion must delete the five-state tactical
solver lifecycle and canonical pre-entry artifact in the same change.
