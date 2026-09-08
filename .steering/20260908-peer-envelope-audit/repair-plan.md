# Complete peer body producer repair

Autonomous implementation authorised2026-09-09. Coordinate slice baseline:
`5cfc50bc685d9ce52c489d7d9962e42ae43ec9c5`.

The measured body overlap and native+0.822mclearance are already established
in `output/20260908-peer-envelope-audit/overlap-witness.json` and its preserved
native program. The prior extraction covers all4kart bodies and GNSS origins.
Add the same witness to package regression before replacing the producer.

Use one explicit `mpc.v2x_peer_body_radius_m`, default1.876mfor this calibrated
local AWSIM, independent of combined planner lateral distance. Nominal radius
is about the actual V2X/GNSS point. Add prediction/position uncertainty once;
the physical verifier adds the ego footprint itself. Recovery reads the same
nominal body parameter, retaining its existing prediction uncertainty semantics.
Remove the obsolete ego-width subtraction and independently sized Recovery
body radius. Do not tune body size to obtain a passing maneuver.

Internal configuration migration: retire
`stuck_recovery.rear_safety.vehicle_radius_m`; use the shared MPCC parameter.
Reject the retired key explicitly so external old configurations cannot silently
retain the under-sized Recovery model. Update local/cloud YAML and specification
examples. V2X topic, message, Domain and evaluation interfaces are unchanged.

Preserve an independently generated3Dconvex hull fixture relative to base_link,
GNSS transforms, all4asset identities and extraction hashes. Test every hull
vertex against each configured nominal sphere without consuming margins.
The existing lateral-spacing test remains a separate planner invariant.

Then build and fullpackage regression, single-source geometry/certificate
lineage checks, and a bounded2-vehicle feasibility run with fixed inputs.
All outcomes remain recorded. If full circles prevent passing/start geometry,
compare feasible representations and their observable orientation contracts;
do not silently use motion heading as exact body orientation. Final integrated
acceptance and submission still follow after this slice.

Rollback: revert this slice only to the coordinate baseline above. Remaining
normal authority stays the single certified7-state controller; no new fallback.
