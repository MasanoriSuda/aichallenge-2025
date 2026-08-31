# Design

## Root cause

The consumer conflated two different facts:

1. an older selected-side artifact can still produce one currently certified
   command;
2. the latest immutable world epoch certifies the selected homotopy.

The first fact is a continuity property.  The second is a tactical feasibility
property.  At decision 2028 only the first was true, while the opposite
same-epoch branch was fully certified.  `selected_authority_available` and the
outer `!production_authority` condition prevented replacement, so a continuity
artifact silently owned tactical selection.

## Corrected authority rule

```text
latest immutable dual epoch
  selected side certified
    -> keep selected side
  selected side rejected + sibling certified
    -> verify active target/generation/intent
    -> verify pre-no-return and replacement budget
    -> current-world revalidate sibling
    -> serialize sibling command
    -> after exact publication, atomically commit sibling homotopy
  neither certified
    -> retain any independently valid current command only as continuity
       evidence; do not manufacture a sibling
```

The sibling is not a fallback synthesized after failure.  It is the certified
other result of the same observation epoch, already computed by the production
dual population.  The change removes an erroneous lifecycle gate and does not
relax physical proof.

## State and identity

`selected_current_world_authority` means the branch bank contains a certified
plan for the live selected side in the same immutable epoch as the sibling.
It deliberately does not mean that an older retained selected plan can still
be revalidated for one command.

The existing publication token remains the only path which changes tactical
side.  Target, Mission generation, intent, live side, source epoch, sibling
side, hard-fault, homotopy establishment, no-return and replacement budget are
all checked again at publication.

## Rejected alternatives

- Extend the retained artifact lifetime: hides the tactical defect.
- Relax wall proof or steering limits: unsupported by the same-snapshot result.
- Add another escape timeout or Mission resume edge: adds lifecycle state
  without addressing the incorrect authority meaning.
- Switch immediately in the worker: violates the single publisher boundary.

