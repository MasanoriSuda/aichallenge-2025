# Design

Extend the frozen observability audit rather than introducing another training
path.  Every sampled state carries:

- immutable sequence ID and source bag;
- original sequence-local sample index;
- stored teacher correction;
- exact v11 input and physical geometry representation;
- nearest admitted normal sequence/sample identity and distance.

For each representation, compute aggregate, per-sequence and final-200-sample
conflict rates against the normal cross-run p50/p95 scale.  The focus sequence
`20260901-130837/d1` is reported explicitly because it is the frozen unseen
generalization sequence used by the current admission Gate.  Known
four-vehicle failure tails are also visible in the per-sequence report.

Replay `LidarPrecontactTeacher(GapTeacherConfig())` with the embedded frozen-base
steering.  Stored teacher labels must reproduce.  On production-normal states,
the replay identifies which teacher reasons contradict the admitted zero
intervention label.  On teacher states inside the exact p50 envelope, reason
counts show which heuristic creates the strongest ambiguity.

This Slice may recommend a later conflict-aware admission experiment only when
known failure tails remain outside the removal region.  It does not perform
that filtering or train a candidate.
