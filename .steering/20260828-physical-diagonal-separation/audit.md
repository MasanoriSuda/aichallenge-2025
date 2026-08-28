# Root-cause audit

## Earliest suspected violation

The dynamic-obstacle convexifier can express only complete longitudinal or
complete lateral separation.  Its `partial_side_escape` exception lowers a
lateral row to an obstacle-free wall witness, although that witness does not
prove obstacle separation.

## Causal chain under test

```text
axis-only obstacle disjunction
  -> no reachable complete behind-to-side transition
  -> partial_side_escape substitutes wall reachability for body separation
  -> SQP can solve an obstacle-penetrating path
  -> exact dynamic proof rejects
  -> production artifact disappears
```

## Deterministic falsifier

If a physical-support candidate using the same replay-world geometry cannot
produce a certified bundle where normalized candidate E did, then the proposed
production repair is not established and must not be promoted.

## Observed result

The falsifier did not occur.  Physical diagonal candidate F produced one
certified left bundle for schedule `1 -> 3`, candidate fingerprint
`16820872117393555423`.  Eleven other left solves were rejected by the exact
dynamic proof, confirming that the proof boundary remains necessary.

## Root-cause conclusion

The earliest demonstrated defect is the axis-only dynamic-obstacle candidate
representation.  A current-world physically certified topology exists but is
not representable by complete behind/side/ahead rows.  The wall-only
`partial_side_escape` branch is a downstream mask, not a valid repair.

## Production deletion gate

Any later production promotion must delete the `partial_side_escape` branch,
its counters and its misleading documentation in the same commit.  Adding the
physical path while retaining the unsafe mask as a fallback is prohibited.
