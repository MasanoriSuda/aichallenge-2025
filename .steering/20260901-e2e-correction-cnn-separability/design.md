# Design

Reuse the current action-separability pipeline and labels.  Add one diagnostic
variant whose feature vector is:

```text
normalized physical scan[750]
+ synchronized wheel speed / 12 m/s
+ embedded frozen-base steering
```

Unlike `static_raw`, this vector is decoded by a trainable 1D CNN that preserves
angular locality.  Four convolutional layers and a bounded adaptive spatial
pool feed a small classifier together with speed/base context.  It does not
reuse or fine-tune the production network and cannot be exported as a runtime
checkpoint.

The normalized physical inputs are not feature-wise standardized; doing so
would make each beam depend on corpus statistics and obscure the local range
geometry being tested.  All other variants retain the existing train-derived
standardization.

Acceptance requires three-seed improvement in balanced accuracy and material
direction, no increase in normal false-material rate, and no peer/focus-tail
regression.  One favorable seed is insufficient.
