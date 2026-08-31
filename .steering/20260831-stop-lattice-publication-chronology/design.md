# Design: Stop-lattice publication chronology

## Root cause

`mpcc_rate_resolved_stop_lattice_shadow::Mailbox` treats
`source_normal_identity.sequence` as a global clock.  That sequence belongs to
the artifact producer.  ShiftOut, Pass and Gate-A producers may allocate from
different sequence domains, so numeric order does not imply publication or
world order.

In the frozen run the ShiftOut source was sequence `954`; the later Pass source
was sequence `295`.  The Pass Stop-lattice solve completed, but the mailbox
classified it as rollback.  The consumer retained the old ShiftOut alternate,
which current-world revalidation correctly rejected as `intent-mismatch`.
With no compatible normal or Stop artifact, authority fell to Emergency Stop.

The causal chain is therefore:

1. Pass normal contingency becomes wall-infeasible under the rule-based Stop
   generator.
2. A seven-state Pass Stop candidate is solved and certified asynchronously.
3. Mailbox compares producer-local sequence `295` against `954`.
4. The physically valid Pass result is discarded as rollback.
5. The stale ShiftOut candidate fails semantic identity join.
6. Normal authority is lost and Emergency Stop interrupts the Pass.

## Change

Use `source_context.decision_id` as the mailbox publication epoch and consumer
watermark.  `decision_id` is assigned by the single control-cycle owner and is
monotonic across Track, Cruise, Follow, ShiftOut, Pass and Return producers.

The full artifact identity remains attached to every result.  Publication
chronology only decides which completed result is newest; it does not authorize
execution.  The existing `artifact::same_identity()` join against the current
published source remains the authority boundary.

## Rejected alternatives

- Retain the ShiftOut candidate through a Pass grace period: hides the
  provenance defect and weakens semantic identity.
- Compare `snapshot_sec`: floating timestamps are not the canonical discrete
  control chronology and complicate equality.
- Reset the mailbox at every phase transition: races with in-flight work and
  discards valid evidence.
- Relax `IntentMismatch`: would allow the wrong semantic artifact to execute.
- Change Stop path gains or wall clearance: the frozen audit proves a feasible
  seven-state Stop already exists.

## Expected effect

A completed Pass result can replace an older ShiftOut result regardless of
local artifact sequence.  If it still fails the exact current-source identity
join, it remains observation-only and cannot publish.  This localizes the
change to asynchronous transport and keeps all physical and authority proofs
unchanged.
