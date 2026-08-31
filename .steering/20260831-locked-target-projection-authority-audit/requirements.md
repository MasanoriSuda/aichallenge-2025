# Requirements

## Objective

Determine why a healthy V2X observation becomes `locked target stale or lost`
during certified ShiftOut execution, and whether target classification or
physical target observability owns the terminal abort incorrectly.

## Frozen snapshot

Run: `output/20260831-173350/d1/autoware.log`

- ShiftOut started at line 957.
- The canonical seven-state ShiftOut artifact remained certified.
- V2X health was Healthy with one received vehicle.
- `locked_seen` alternated to false near a hard curve.
- ShiftOut transitioned to Recovery at line 1257 solely because the locked
  target was classified stale/lost.
- Episode wall/corridor reserve was 6.07 m and maximum lateral acceleration
  was 3.24 m/s2; this was not a physical wall/lateral-acceleration failure.

## Constraints

- Do not change V2X timeout, target hold, projection distance, clearance,
  solver tolerance, or fallback duration.
- Do not add a target-loss grace or Mission resume rule.
- Preserve position-jump and course-branch discontinuity hard rejection.
- Compare raw physical observation, common-course projection, target
  provenance, current certified artifact, and phase authority at one epoch.

## Acceptance

- Root cause is classified as observation loss, projection/classification
  loss, certificate mismatch, or genuine physical infeasibility.
- Any repair makes physical target observability and tactical relevance
  distinct concepts with one explicit authority boundary.
- Static tests cover hairpin projection loss without weakening position-jump
  or wrong-identity rejection.
- Dynamic verification records whether a healthy observed target still causes
  terminal Recovery while a certified current-world artifact exists.
