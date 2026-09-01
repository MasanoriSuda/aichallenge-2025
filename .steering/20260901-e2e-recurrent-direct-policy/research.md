# Primary-source comparison

## End2Race

- Paper: <https://arxiv.org/abs/2509.16894>
- Reference implementation: <https://github.com/michigan-traffic-lab/End2Race>

The paper and public implementation preserve LiDAR angular features through a
learnable per-beam spatial-pressure transform, add ego speed, and train a
unidirectional GRU over complete racing scenarios.  This Slice adopted those
principles as an offline comparison, while retaining this repository's
750-beam scan, steering units, immutable split and validation gates.

The reported F1TENTH Gym scenarios and non-reactive opponent are not equivalent
to AWSIM.  Reported paper metrics therefore were not reused as local admission
thresholds.

## TinyLidarNet

- Reference implementation: <https://github.com/CSL-KU/TinyLidarNet>

TinyLidarNet remains the frozen production baseline.  The experiment did not
alter its checkpoint or runtime authority.  Its validation advantage over the
new direct recurrent policy is part of the rejection evidence.
