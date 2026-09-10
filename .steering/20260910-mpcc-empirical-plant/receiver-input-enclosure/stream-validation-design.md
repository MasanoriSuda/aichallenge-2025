# Validate applied samples during prediction

2026-09-11 JST, baseline/rollback4eb19997. Diagnostic dev2-r19captured the
previously missing slow alternates: D1decision854/source182 takes187.114ms,
including169.332msapplied proof; D2decision836/source145 takes139.706ms,
including118.292msapplied proof. Both exact requests are marked present.
Native r84replays unchanged D1in135–137ms and D2in88–90ms. Both finish numerical
prediction successfully, then reject a modeled state: D1at8.459999825,
D2at8.024999825, versus publication7.794999825. Outer SteeringUnreachable
masks AppliedProgramUnavailable in the feedback bundle, not a separate solver
failure. Frozen libraries/sources live under stream-validation-before.

Producer: certify_terminal_stop first predicts the whole response population
to rest, then iterates its states for unchanged bounds/wall/peer/Follow checks.
Once any necessary condition fails on an earlier sample, later prediction
cannot make that same candidate certifiable. Deferring that check spends most
of the callback on an already invalid candidate and contributed to r18's lost
publication window. This is an evaluation-order defect, not evidence for
raising time budgets, dropping an input member or removing the candidate.

Keep the original four-argument numerical API and full continuous-domain
primitives. Add a checked overload with a validator for the publication body
and each subsequent swept sample. A false validator aborts with a distinct
ValidationRejected result and no tube. Valid input still runs until complete
modeled rest. Prefix samples before publication are still propagated without
changing their original history. Publication desired-input assignment and
sample clocks must stay bit-for-bit identical to the old tube.

The existing applied certificate owns the same bounds, map, full CA1peers,
Follow projection, nominal fingerprint and command checks. Invoke its current
check as each sample arrives, before computing later samples. Retain its first
failure reason/time/peer; no partially validated tube or certificate can escape.
Verify nominal/source/program identity before checking and bind the full tube
only on complete success. The default no-validator path must stay unchanged.

Same-world r19D1nominal A/B-left/C-left/D-left succeeded, opposite-side arms
rejected; complete-rest first attempt rejected and another accepted. This is
nominal feasibility only, not an applied certificate or runtime guarantee.
Record each arm and attempt without claiming physical infeasibility.

Tests: early rejection yields no partial result and performs no later visitor
calls; accepting visitor visits publication then all original future samples
and produces the same complete tube. Existing bad-world/peer/Follow/identity
certificate tests remain. Exact slow replays must keep their rejection and
first rejected time while removing computation beyond it. Recheck accepted
r17case, signed r16case, build/package and source/native numerical suites.
Then fresh integrated runs, remaining peer failure audit and M4–M6 acceptance.
No margins, tolerances, solver budgets, model constants or rates change.

## Duplicate solved-suffix evaluation

Streaming alone preserves the exact first rejection but still takes27.8–29.3ms
for D1and23.2–25.5msfor D2(native r85). Both saved plans require terminal body
rest. Their result reports zero lateral-reference attempts and selects the
already solved complete Stop suffix. That branch never reads the supplied
terminal lateral reference. Nevertheless evaluate() retried the track reference
after the applied certificate rejected, repeating identical controls/world and
both physical/prediction passes. Return that solved-suffix result directly;
retain the distinct lateral-reference alternative when a Stop was actually
constructed from a reference. This removes duplicate work, not a candidate.
Existing Stop/feedback/reference regressions and the exact r19replays must
retain outcomes and first rejected times. r84/r85are preserved as before cases.

## Verification

Buildr43:26packages/4m27s. Focused24vehicle+81retained cases and package
r36:2432records/62groups pass with zeroerrors/failures/skips. Native r86
D1cost13.97–15.09ms and D2cost11.81–13.08ms; exact original physical reason and
first rejected time retained, no proof returned. Native r87/r88 preserve accepted
r17certificate/materialization/join and signed r16Stop join, and reject the real
r18history gap. Original r19moving failure requests still reject wall(D1) and
peer d1(D2) at the same times. Default4argument ABI and four source/native
response suites pass. Frozen before/after libraries and source inputs are
hashed in [evidence](stream-validation-evidence.json).

Diagnostic precedence: physical early rejection now reports numerical
ValidationRejected(8), instead of successful full numerical prediction(0).
Invalid-world checks run before numerical prediction. Necessary requirements
are unchanged; this is not evidence that combined invalid-input errors retain
the old presentation precedence. Full callback/publication timing, all moving
failures and M4–M6 remain unaccepted; freshdev2-r20 is next.
