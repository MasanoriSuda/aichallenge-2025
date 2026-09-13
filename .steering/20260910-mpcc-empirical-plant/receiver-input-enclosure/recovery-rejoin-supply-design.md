# Rejoin candidate supply while the final command waits

2026-09-13. Baselineea14f80a. Authorized autonomous completion/local commits continue.
Rollbackea14f80a; retain the strict clearance, confirmation and actual launch fixes.

single-r14/R677/R678:4234callbacks,max21.132791ms,no25msexcess or actual publication
window violation;355normal sends through901/source516. Publicspeed reaches1.056758mps.
Declared1and fresh complete V2X work; forced_candidate/v2x_override remain0. Recovery
performs bounded ForwardCreep and reaches10LowSpeedRejoin spans, but none publishes
a normal Rejoin; final SafeStop is attempt_limit_reached. No Start/laps/acceptance.

R681records the first blocked producer. During Rejoin963/12.404999722and subsequent
callbacks, normal_submitted409and worker409stay fixed, draft=available but successor=
normal-successor-disallowed. The final emergency command waiting for a certified
Rejoin sets recovery_command_active. MPCControllerCpp::control then blocks both
submit_scheduled_post_publication and record_rate_resolved_publication_successor.
A certified candidate is required to remove the override, but the override forbids
producing/joining that candidate. Source wall infeasibility is not established by
these unattempted requests. Attempt limits/physical clearance are downstream guards,
not the cause of source409stall. Preserve them.

## Boundary and alternatives

A normal planning request must bind the actually serialized predecessor; it does
not require that predecessor to have normal authority. Keep final publication and
current-world/immutable identity proof strictly unchanged. Separately permit normal
planning when Rejoin was requested at solve and the current active, actuation-allowed
Recovery output still has LowSpeedRejoin state and action. A first transition from
another phase cannot submit its old intent; HoldStop/SafeStop/reverse/gear/shadow/
blocked supervision cannot use this permission. Feed the same eligibility to both
post-publication scheduling and serialized source submission. Existing control and
fallback checks remain. This permission owns computation only, never publication.

Comparison: keeping the current shared publication/planning gate deterministically
starves all10captured spans (rejected). Separating planning from publication at the
existing serialized boundary preserves each authority and allows current-world
certification (chosen for regression). Direct Rejoin feedback was already retired
in9950and would create a second normal owner (rejected). A one-shot entry solve or
longer wait cannot establish replenishment/current-world joining (not chosen).
No margins, weights, retry budgets, timeouts, tolerance or physical model changes.

Verification: fail a source wiring regression on both current gates; native policy
cases for the captured waiting state, entry/interruption and inactive supervision;
source/architecture regression; make autoware-build130and package117controller tests;
review/seal/local commit; only then a source-fixed Recovery diagnostic. Keep all
existing certified publication gates and publish-before/after checks in the test.

## Independent actual publication deadline failure

dev2-r80/R679/R680:both nodes receive declared2; no empty-world publisher. D1receives
72map-frame arrays containing d2, D2receives65containing d1; positive ordered source
stamps, medianperiod~0.066666s. The campaign stops on D2decision593actual publication
window violation: nominal2.604999947,before2.629999941,after2.634999941,deadline2.629999947.
Publish call costs2.853515ms. Successful-publication rows alone omit this failed
post-publication send: R679window_violations=[]is not evidence of zero actual violations.
R681corrects that diagnostic scope and seals the post593snapshot. Neither Rejoin
planning nor launch repair resolves this timing failure. Preserve the original25ms
window; exact replay/architecture audit is required, not a larger timeout or hidden
warning. Runtime stopped and userJSON/originalDLL preserved. All M4–M6still open.
[Evidence](recovery-rejoin-supply-evidence.json).

## Implemented boundary validation

R683fails the original source wiring; R682was only an unrelated host plugin
collection failure. R684test compilation omitted two using declarations; no tests
ran. R685corrects the test names and all151Core cases pass. The old source assertion
for the retired shared gate is replaced by separate planning and unchanged final
publication assertions;118source/architecture tests pass. Build130all26passes with
frozen inputs, package117: Summary: 2637 tests, 0 errors, 0 failures, 0 skipped. No numerical/physical/clock change.

R686rebuilds the exact original source/domain/current history for D2decision593,
then appends the separately captured actual send with exact source/wire/recorded
epoch. Before guard passes, after guard rejects. The actual out-of-window send
remains recorded; it is not certified retrospectively. Send-transaction timing
still needs a structural solution. After review/local commit, single-r15may test
Rejoin supply independently under the same strict diagnostic stop conditions;
this does not clear the failed multi-vehicle timing gate or grant race acceptance.
[Validation](recovery-rejoin-supply-validation.json).
