# Peer separation without enclosing-rectangle corner inflation

2026-09-11 JST. Baseline and rollback: 5f673517. M4–M6 remain unfinished.

Dev2-r22 first fails at D1 decision1024, 2.73m/s, WP35, wall time
1789072121.843017602. Captured source338 finally rejects wall12.999999729
under the last tracking reference. Its first common constant-steering Stop
instead passes walls and rejects peer d2 from13.559999729. Original peer
minimum is −0.101961831m. Previous1023/source365 is the actual separate
publication; the recorder correctly rejects association with source338.

Native r105 materializes that previous programme with original current
observation/history: it also rejects updated peer d2 from13.514999729.
Thus merely retaining the prior programme remains insufficient. The old
peer generation201 changes to202: velocity/acceleration prediction changes
with the received sample. No peer model or filter change is justified here.
D2 later1083/2.63m/s also rejects a peer; it occurs after D1's Emergency and
is not the independent first failure. Preserve that chronology.

The applied proof converts the entire x/y/yaw pose population to one enlarged
oriented rectangle. That rectangle includes unused corners outside the actual
union of possible original footprints. r106 compares separating half-spaces
on the same complete state boxes, original footprint/margin and fully swept
CA1 peer circles.128 directions are candidate proof directions, not sampled
vehicle responses. Every common programme sample separates through full
rest13.614999729, minimum0.09378643m. This is a geometric enclosure comparison,
not a narrower input population or acceptance threshold.

R107 replaces the direction search by one direction from the enclosing
rectangle's nearest point to the peer centre. Every sample still separates,
minimum0.02546312m. R108 isolated implementation uses that one direction only
when the existing rectangle test rejects. Outward interval arithmetic bounds
the projection of all four original margin-expanded corners across every
x/y/yaw value; the normal's bounded norm accounts for the unchanged peer
radius. A nonnegative separation lower bound independently proves clearance.
Failure to find a separating plane retains the original rejection.

R108 accepts the exact D11024/source338 through all original nominal, applied,
production adapter, Stop materialization and same-epoch joining gates,11.82ms
isolated. Full programme has303 samples,2commands, unchanged rest13.614999729.
Minimum recorded clearance0.006229782m includes earlier positive rectangle
bounds. D21083 still rejects peer d1 at15.014999681. Replay time is not25ms
live acceptance. One-direction geometry introduces no retry, extra normal
controller, grace, input/model/rate/margin/solver change or new normal authority.

Producer: applied-program peer clearance's single enlarged rectangle.
Replace its exclusivity with the second valid geometric separation proof;
keep all current-world checks and strict certificate/programme identity.
Neither finite sampled trajectories nor the nominal-only complete-rest solve
can supply this authority. Counterexamples, contact/boundary and randomized
native geometry tests must validate the new bound, followed by full build,
package and original-source regressions, then fresh dev2 and remaining M4–M6.

Same-world persistentA nominal solve passes20.3694ms. B/C/D/G lack a current
canonical target and are inconclusive. Complete-restY first reaches iteration
limit111.065ms and then passes24.0833ms; both outcomes remain recorded. No
all-method physical infeasibility claim. This repair leaves peer-model update
uncertainty, actual publication-clock agreement and recursive Stop acceptance
open for the integrated campaign.

## Verification

Build r46 exposed the new independent geometry test's missing recovery_footprint
link dependency. The test target now declares it; build r47 passes26packages
in13.4s. Package r39 passes2437records/62groups with no errors, failures or skips.
Fresh r109/r110 original-source replays pass exact D11024 full certification,
production, materialization and join, all earlier positive regressions, while
real r18history gap and later D21083peer rejection remain correctly rejected.
The shared vehicle integration kernel is unchanged. See
[evidence](peer-separation-evidence.json). Freshdev2-r23 and M4–M6 remain required.
