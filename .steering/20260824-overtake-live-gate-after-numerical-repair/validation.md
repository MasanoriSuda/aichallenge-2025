# Validation

## Commands

```bash
make dev2
make down
```

Frozen source/config commit: `8ff1a9e`

Evidence directory: `output/20260824-031752`

## Domain 1 live evidence

- Overtake episodes: 2
- Canonical shadow cycles: evaluated 322, eligible 256
- Fresh complete/stored canonical plans: 198
- Fresh physical rejects: 2 hard-wall contacts
- Retained attempts/certified selections: 58/0
  - cursor rejected: 40
  - course-frame unavailable: 18
- Extended production cycles: eligible 322, successful 220, fallback 102
  - circuit skip: 92
  - solve failure: 4
  - requalifying: 6
- Exact downstream `stage=constraint_check` failures: 0
- Callback cycles/overruns: 5499/28
- Callback maximum: 263.152 ms for a 25 ms period

## Joined authority evidence

The change-aware final trace emitted 16 Overtake-episode states. None satisfied
the canonical command contract:

- 2 physically certified five-state states:
  `missing-canonical-command-identity`
- 14 states:
  `legacy-normal-bypass`
- 7 of the 16 emitted states used an Overtake wall-admission hold source.

The trace emitter suppresses unchanged cycles, so these 16 records are state
changes rather than total control-cycle counts. The shadow and runtime windows
above provide the cycle totals.

## Gate result

**Rejected for Overtake production authority promotion.**

The numerical scaling repair is accepted: the prior exact physical-row
constraint failure did not recur. The newly uncovered first invariant failure
is the split Overtake producer/publication lifecycle. The next change must
repair that lifecycle and same-formulation continuity before publisher
promotion.
