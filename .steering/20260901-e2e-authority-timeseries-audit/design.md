# Design

Replay the v2 spatial candidate over the immutable recurrent sequence derived
from `/output/20260901-180313/d1/rosbag2_autoware`.  Compare its residual with
`LidarPrecontactTeacher - candidate3` in the final 200 samples.  Repeat the
same replay for the independently trained v3 DAgger candidate and the v4
globally authority-bounded candidate.  This separates a data-distribution
change from a runtime-bound change without granting either candidate runtime
authority.

Preserve time order.  Split the target into contiguous left, neutral and right
segments and report the first five-sample sustained model response after each
material segment begins.  Evaluate model output after diagnostic symmetric
bounds without changing the recorded command.

The report is causal: the source sequence ends one second before the first
confirmed LiDAR breach.  It can determine direction, delay and clipping, but
cannot by itself prove vehicle dynamics after the excluded boundary.  A
candidate may advance only when its transition timing improves without using
the failed authority sequence for training; authority magnitude must remain a
separate, later closed-loop decision.
