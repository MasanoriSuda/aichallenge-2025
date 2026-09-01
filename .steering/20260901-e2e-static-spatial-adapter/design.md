# Design

```text
physical 750-beam LiDAR
  -> frozen candidate3 conv1..conv5
  -> full spatial map
  -> LayerNorm + small MLP
  -> left / neutral / right probabilities
  -> signed bounded magnitudes
  -> correction only (offline)
```

The base network is embedded for provenance but cannot receive gradients.  A
zero-initial signed mixture cancels left and right corrections exactly.  The
candidate learns `successor_teacher - persisted candidate3 steering`; it does
not relearn track following.

Sequence-balanced sampling prevents short competition-failure prefixes from
being diluted by long successful runs.  Direction class weights are computed
from train sequences only.  Validation, peer-d3 and an independent normal
TinyLidarNet corpus are evaluated without checkpoint selection leakage.

This Slice does not add a ROS subscriber or runtime model path.  A passing
artifact authorizes a later shadow Slice; it does not authorize production.
