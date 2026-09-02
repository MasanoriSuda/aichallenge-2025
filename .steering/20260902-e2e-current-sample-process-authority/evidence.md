# Evidence

## Static acceptance

- Host focused recurrent/core suite: `53 passed`.
- Host full TinyLidarNet controller suite: `92 passed`.
- Host E2E launch contract: `3 passed`.
- `make autoware-build`: 25 packages completed successfully.
- Docker installed-space TinyLidarNet suite: `92 passed`.
- Docker installed-space E2E launch contract: `3 passed`.
- Python compilation and `git diff --check`: passed.

The core no longer evaluates recurrent steering authority through its local
NumPy model merely because the authority flag is true.  An explicitly bound
current-sample evaluator is mandatory.  Without it, the recurrent result is
classified as an inference error and the already-valid spatial production
command is returned without recurrent authority.

The production ROS binding starts an exact-SHA subprocess, verifies the
self-described runtime contract, constrains child OpenBLAS to one worker and
binds that evaluator before subscriptions start.  The child reply must match
the current private request sequence.  Authority-disabled observation retains
the accepted asynchronous latest-wins path.

Dynamic A/B acceptance is still pending.  Packaged defaults and the production
checkpoint remain unchanged.
