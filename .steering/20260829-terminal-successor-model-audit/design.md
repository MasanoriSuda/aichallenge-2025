# Design audit

## First causal observation

The first authority loss in `output/20260829-164506` is not a progress or
steering continuity failure.  It is the terminal-successor proof:

```text
normal continuation: current-stage prefix clear
terminal Stop suffix: wall collision
result: terminal-contingency-unavailable
```

## Initial hypotheses

### H1: certified and published Stop lateral policies differ — confirmed

`build_stop_contingency()` fixes `steering_rate_radps = 0`, so it proves max
braking while holding the current steering.  `canonical_normal_emergency_stop()`
instead selects `TrackReferencePath` while moving and rate-limits steering
toward that path target.  The failure log confirms production selected
`lateral=track-reference-path`.

The audit also found that the first unavoidable publisher interval replayed
acceleration but discarded the serialized steering-rate command.  Therefore
both halves of the old proof differed from the causal wire sequence.

The retained proof rejected a hypothetical hold-steering path, then production
published a different unproved path-tracking Stop.  This was a
model/certificate mismatch rather than proof that the real Stop path was
blocked.

Confidence: high.

### H2: a genuinely feasible terminal successor becomes infeasible as the
vehicle diverges from the planned ShiftOut

Support: at decision 1710 the physical pose join error is 0.73 m and 0.24 rad.

Refutation: evaluate the same frozen current state with both the certified
hold-steering policy and the actual path-tracking Stop policy.  If only the
latter is clear, policy mismatch is causal; if both collide, the physical
state is genuinely unrecoverable under this Stop family.

Confidence: medium.

### H3: candidate/Gate-A and retained wall worlds differ

Support: the result changes roughly 0.19 s after first publication.

Refutation: compare immutable wall-grid identity, footprint, swept step,
course geometry and control-origin timestamps at Gate A and retained failure.

Confidence: low to medium until provenance is logged together.

## Correction boundary

Do not simply remove terminal Stop certification.  A partial prefix needs a
real safe successor.  The correction must establish a single Stop lateral
policy artifact which is both:

1. simulated and wall/dynamic certified from the current control origin; and
2. the policy the external Stop publisher serializes if normal authority is
   unavailable.

## Implemented contract

- One pure `resolve_stop_path_tracking_command()` owns moving-Stop lateral
  feedback, lateral-acceleration limiting and steering-rate limiting.
- Terminal proof first replays the exact already-serialized acceleration and
  steering rate for one publication interval.
- Its braking suffix then samples the current course geometry every publisher
  interval and executes that shared Stop law with maximum braking.
- `canonical_normal_emergency_stop()` calls the same pure law.  At-rest hold
  and invalid-input neutralization remain explicit separate actions.
- The trace records policy identity, exact rejected sample, causal hand-off
  steering and terminal steering.

No wall, solver, continuity, lateral-acceleration or timing threshold changed.

## Dynamic conclusion

In `output/20260829-172407`, D1 produced nine partial-normal terminal Stop
proofs.  All nine were certified; no `terminal-contingency-unavailable` was
observed.  In particular, a normal continuation rejected as
`invalid-lateral-bounds` at decision 1130 still obtained an exact accepted,
wall-clear and dynamic-clear terminal Stop successor.

Later `continuation-rejected` events did not reach partial-prefix proof because
the current stage itself had already left the lateral bounds.  They are a
separate upstream current-stage/candidate problem and are intentionally not
patched in this Slice.
