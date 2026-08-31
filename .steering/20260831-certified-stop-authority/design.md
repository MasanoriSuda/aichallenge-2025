# Design: certified Stop authority

## Root cause

The current-world Stop lattice returns a `CertifiedPlan` derived from the
active ShiftOut/Pass snapshot.  Its trajectory is a maximum-braking Stop, but
its immutable problem/command identity necessarily names the source tactical
intent.  The production bridge copied that plan into the normal retained
result and then called the ordinary normal publisher without recording that
the selected authority was a terminal contingency.

Consequently:

```text
certified Stop geometry
  -> CanonicalNormalCommand(intent=ShiftOut)
  -> published_authority_intent=ShiftOut
  -> normal execution ledger remains live
  -> old normal candidate may rejoin next callback
```

The Stop solver and proof were correct; publisher semantics were not.

## Change

Carry one non-geometric fact through the retained evaluation:

```text
certified_terminal_contingency_selected
```

When true:

- keep the certified plan's internal source intent and exact command unchanged;
- resolve the external published authority intent to `Stop`;
- do not promote or record that plan as normal execution evidence;
- record `Stop` in the canonical publisher ledger;
- let the existing atomic admission retain Stop until a fresh normal plan
  passes current-world proof.

This separates proof provenance (where the Stop was built) from authority
semantics (what command class crossed the publisher).  It does not invent a
second controller or a new Stop trajectory.

## Rejected alternatives

- Add a Stop hold timeout: hides the missing authority identity with a lease.
- Tune wall margin or brake limits: the current-world Stop already certified.
- Treat the no-escape snapshot as a candidate-generation defect: exhaustive
  C/D comparison found no forward Bundle.
- Re-label the immutable problem as Stop: breaks its source fingerprint and
  target/homotopy provenance.
- Use the generic emergency Stop instead: discards the already certified
  lateral/longitudinal Stop trajectory.
