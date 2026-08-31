# Results: Stop-lattice publication chronology

## Frozen failure

In `output/20260831-091516/d1`, decision 1832 had entered `Pass`. The
rule-based terminal Stop was rejected by exact wall proof. A previously
published ShiftOut Stop-lattice result from local artifact sequence 954 was
still visible, but it failed the current semantic join as `intent-mismatch`.
The later Pass producer used local artifact sequence 295; after it completed,
the mailbox counter changed to `rollback=1` and authority fell to canonical
Emergency Stop.

The same immutable snapshot was evaluated without changing constraints:

- persistent and stateless rule-based terminal Stops failed exact wall proof;
- all 128 lateral-only Stop targets failed exact wall proof;
- the seven-state Stop control lattice passed the unchanged SQP, exact wall,
  exact dynamic and terminal proof chain.

Therefore the state was physically stoppable and the missing authority was
not caused by clearance or solver settings. The physically valid result was
lost at asynchronous publication.

## Structural correction

The Stop-lattice mailbox now orders completed work by the single control
owner's `source_context.decision_id`. The artifact `sequence` remains part of
the exact producer identity, but it is no longer treated as a clock across
ShiftOut, Pass and Gate-A producers. The consumer watermark uses the same
decision chronology. Exact current-source identity matching remains
mandatory before a result can become normal authority.

Telemetry and types now name this event `decision-rollback`, avoiding the old
implication that producer-local artifact sequence is globally ordered.

## Static verification

- source contract: `94 passed` with third-party pytest plugin autoload disabled;
- `make autoware-build`: all 25 packages passed;
- focused GTest `LiveStopShadowMailboxUsesDecisionChronology`: passed;
- full `multi_purpose_mpc_ros` CTest: `59/59` passed.

The regression test accepts ShiftOut `(sequence=954, decision=1736)` followed
by Pass `(sequence=295, decision=1832)`, and rejects a late completion
`(sequence=1200, decision=1800)`.

## Dynamic verification

Bounded `make dev2` run: `output/20260831-093415`.

D1 completed 65 Stop-lattice jobs and published all 65:

```text
worker=submitted:65/started:65/completed:65
mailbox=published:65/invalid:0/rollback:0
```

The run exercised two `Idle -> ShiftOut` transitions and one atomic
`ShiftOut -> Pass` transition. No chronology rollback was observed across
the producer transitions. The first current-world alternate visible at the
Pass boundary was still an older ShiftOut result and was correctly rejected
as `intent-mismatch`; a later Pass-compatible result reached comparison but
was rejected as `steering-unreachable`, not discarded by mailbox chronology.

This is the expected separation of responsibilities: the transport defect is
closed, while candidate reachability remains explicit evidence for a later
root-cause Slice. No grace period, fallback, solver tolerance, clearance or
production-authority rule was added.

## Classification and next boundary

The frozen failure is classified as **offline succeeds but live fails:
scheduling/lifecycle defect**. The chronology defect is closed. The next
failure family must freeze the `steering-unreachable` Pass/ShiftOut snapshot
and compare persistent, stateless, rough/lattice and nonlinear feasibility on
that exact world; it must not reopen this Slice through parameter tuning.
