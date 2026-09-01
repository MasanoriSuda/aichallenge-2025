# Design

The precontact teacher computes its residual from both the current LiDAR scan
and the frozen base steering.  The static spatial student previously received
LiDAR and speed only, even though its target was `teacher - base`.  Therefore
similar scans with different base steering produced conflicting labels that the
student could not represent.

Add the immutable embedded base steering as one scalar adapter feature.  It is
computed from the same LiDAR frame by the already embedded, bit-checked frozen
base and introduces no ROS topic or non-ML lateral owner.  Existing checkpoint
shapes remain unchanged unless the explicit feature flag is selected.

The failed wheel-v4 candidate remains rejected.  The conditioned candidate is
offline-only until it passes the existing bounded evaluator, held-out failure
audit, independent normal leakage gate and two-seed NPC closed-loop gate.
