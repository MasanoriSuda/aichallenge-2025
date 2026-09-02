# Design

This Slice tests a teacher source, not a runtime fallback.  The E2E inference
contract remains 2D LiDAR and wheel speed to ML-owned lateral control.  During
training, a privileged MPC/MPCC expert may generate steering labels from a
complete trajectory; the resulting student still receives only allowed E2E
inputs at runtime.

The experiment deliberately reuses `make e2e-peer-audit-mpc`, whose Makefile
boundary removes TinyLidar checkpoint, residual, recurrent, authority and
teacher-mode overrides from every MPC container.  This avoids attributing an
E2E artifact to the expert.

Admission order:

1. unique runtime controller identity;
2. completed expected laps and zero penalties;
3. no post-start low-speed or positive-acceleration stall;
4. finite scan/control streams with the existing sync contract;
5. only then consider `label_source=mpc` extraction.

If the expert fails, the result is evidence that the existing planner cannot
label this world reliably.  No student training is attempted and the next
choice must be a different teacher/scenario, not relabeling the failed suffix.
