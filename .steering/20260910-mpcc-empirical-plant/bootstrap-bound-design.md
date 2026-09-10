# Initial-state equality is not the retained physical corridor

Baseline/rollback99fccb87. Autonomous M1–M6 authorization continues; no push.
Dev2-r9 stops at initial rest after Ready on both vehicles. Normal candidates
are solved and available but their bootstrap continuation repeatedly rejects
`initial-lateral-bound-rejected`. The run is rejected, externally interrupted
through its owned harness, with normal teardown/artifact restoration.

Producer: shadow `build_execution_artifact` copies final QP state0 lower/upper
into physical lateral corridor bounds. The adapter correctly fixes x0 to the
source observation; that equality must stay in the QP and semantic initial
state. A retained candidate is reprojected from its current prospective prefix,
so its fresh initial lateral coordinate differs by millimetres even at rest.
Treating the old x0 equality as a physical road boundary rejects that valid
state before any swept wall/peer proof. Extending a rest hold or ignoring the
prospective packet would hide the defect.

Evidence: r9D1decision840/source294 and D2decision819/source294 capture the
first Cruise transition with a still-Track artifact. Exact native replay
therefore keeps their `intent-mismatch`; comparisonr1 is inconclusive for the
later physical rejection. R2 explicitly evaluates each artifact under its own
Track intent in both arms, without claiming a real Track adoption. Original
initial equalities1.034464984and−2.740486784reject fresh lateral1.032811999and
−2.741825121. Replacing only initial physical bounds by the already sealed
initial map support, resealing the artifact identity and recertifying its native
source, admits both current Requests and matching packets. Terminal peer
clearances0.632794517/1.072191784m. All future corridor values, body/proposed
packet/history/obstacles and proof budgets remain fixed. Zero solves.
Runtime D1decision2207 and D2decision1589 separately name the same physical
rejection; their complete Requests were not captured by the first-event recorder.
Do not relabel the first intent-mismatch snapshots as those later decisions.

Repair the artifact producer to carry the source's immutable physical map
support at its initial progress coordinate when that geometry is available.
The source QP x0 equality and semantic body state remain exact; future bounds,
all native dynamics, swept wall/all-peer/terminal proofs and final packet binding
remain unchanged. Missing physical map geometry in historical/unit diagnostic
snapshots retains their original bounds. Production's existing full terminal
geometry requirement remains in force. Do not enlarge physical support or
add tolerances if the source lies outside it; preserve and investigate rejection.

Acceptance: failing native regression for a2mm fresh body displacement, unchanged
QP initial equality, rejection outside the physical corridor; reproduce source
builder on sealed r9 with both exact and named comparisons; package tests/build,
quality checks, local commit, fresh dev2 then M4–M6. Current-speed negative sensor
noise at rest also appears as intermittent invalid-current-state; it is separate
from the consistently reproduced initial-lateral rejection and is not claimed fixed.

## Implemented candidate and local evidence

The shadow artifact producer now samples its already sealed physical map at the
semantic initial progress, keeping every later bound and the QP state0 equality.
An unavailable/out-of-range sample or invalid supplied geometry rejects. The
map is not enlarged to include a rejected source. No retained proof is skipped.

Buildr22 (test-only) succeeds26packages. The new native regression fails on the
old producer: initial bounds remain0/0 and a2mm displacement has no continuation.
Buildr23 fails at compile time on an optional geometry dereference; this is
retained as a preparation failure. Corrected buildr24 succeeds26packages/26.1s;
testsr18passes2402entries/62CTestgroups/0errors/failures/skips in28.59s. The
regression also checks unchanged QP/semantic initial equality and physical
corridor rejection at0.81m when the upper map bound is0.8m.

Native comparisonr4/toolr23 preserves exact intent mismatch and the named
zero-solve physical-bound comparison. Separately, it invokes the production
SQP pipeline on each captured current source and the exact inspected Track294
source. Both inspected source builders solve and pass physical proof, retaining
the original semantic/QP x0 and carrying initial map bounds. Current D1source840
also solves/proves; current D2source819 remains solver-rejected at4000iterations
before physical proof, as inr3. Keep that rejection; do not retune its budget.
R3's `solver_invocations:1` means one external SQP pipeline invocation per car,
not one OSQP iteration/call; toolr22's copied compile-purpose text still says
zero solves, and is superseded by this explicit distinction. R4 correctly
separates the zero-solve comparison from two named pipeline invocations.

All r1–r4 results are retained. The intermediate diagnostic drivers are recorded
by compile hashes and binaries; the final driver and actual included reader are
preserved with this candidate. Fresh committed dev2-r10 is next. Local evidence
does not close race, timing, M4 scenarios or M5/M6 acceptance.
