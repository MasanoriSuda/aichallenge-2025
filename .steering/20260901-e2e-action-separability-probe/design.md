# Design

The label is successor-teacher correction relative to frozen candidate3:

- left: correction <= -0.02 rad;
- neutral: absolute correction < 0.02 rad;
- right: correction >= +0.02 rad.

Variants:

1. `static_fc3`: admitted 10-dimensional policy embedding plus speed;
2. `static_conv5`: frozen conv5 spatial map projected deterministically to 128
   dimensions plus speed;
3. `temporal_conv5`: the same spatial feature plus 1/8-step feature and speed
   differences.

Each variant trains only a small weighted three-class MLP.  This is deliberately
weaker than a candidate controller: if a probe cannot separate the actions, a
new recurrent regression head should not be trained.  Conversely, a gain only
authorizes a later bounded adapter experiment, not production promotion.

Frozen CNN features are computed once per immutable sequence and reused by all
three comparisons.  The probe RNG is reset before each variant, so comparison
results do not depend on the order in which variants execute.
