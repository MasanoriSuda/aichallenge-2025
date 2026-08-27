# Design

The canonical controller performs an initial QP, a mandatory current-problem
relinearized QP, and current-world wall/dynamic certification.  During the
measured overtake windows, full 20-stage problems dominate the callback tail;
the optional post-refinement path was already inactive.  Reducing the horizon
from 20 to 16 therefore removes work from every mandatory stage without
changing the model, objective, constraints, or authority graph.

Sixteen stages retain the existing variable stage duration and reachable
horizon resolution.  Current-world obstacle and wall proofs remain exact over
the complete executable horizon.  Tactical candidate generation retains its
separate long-distance lookahead, so this experiment does not shorten the
course-level side search.

The comparison records runtime tails and phase outcomes rather than fastest
lap.  This is a solve-cadence/horizon experiment, not a performance-weight or
clearance tune.
