# Scheduled context adoption (baseline daf40d09)

Continue authorized M1–M6 without confirmation. R46 failed after Ready: D1
repeatedly rejects SemanticChanged with fresh certified Cruise sources, unchanged
stage geometry and matched actual prefix; D2 has no post-Ready scheduled source.
No lap or moving acceptance. Independent current proof and actual indexed sends
work before Ready. The scheduled physical proof improvement is not whole-task
completion. Protected artifacts were restored after scoped diagnostic SIGINT.

Observed distinction: current Ready Cruise requests use dynamic obstacle side0;
solver candidate snapshots use explicit sides-1/+1. The final immutable D1
source captured at shutdown also has side+1. These are different frame times,
so they are not falsely presented as one exact simultaneous rejection payload.
The worker chooses the first physically certified source, while final context
checks require side equality. Unresolved-versus-selected candidate semantics is
a producer/selection hypothesis, not a reason to relax check_current_context.

Detection gap: the first active recorder stored expected old Track/session
revocation at D1dispatch843/D2dispatch823, consuming the stationary slot before
persistent fresh Cruise mismatch. Add an independent bounded active semantic/
geometry recorder and preserve the worker's original proposed context alongside
its original plan and current request/context. Normal authority, candidate
selection, physical proofs, all parameters, rates and expiry remain unchanged.
Next diagnostic r47 must seal the first exact conflicting input pair, then
compare candidate context representations and repair the responsible producer.
D2's saved first Cruise refinement QPs are separate evidence; solver failure
alone does not prove physical infeasibility or justify iteration/weight changes.

Remaining: current-source selection, actual Stop/rest/restart, all M4intents,
Mission/sibling/Store/Recovery/Rejoin/Boost/async, M5same-final-HEAD repeated
single/dev2, dev3/dev4sixlaps/gates, M6same tar/image/eval. Rollback: daf40d09.

Observer validation: buildr87passes26packages; testsr75passes2530records in
64groups, zero errors/failures/skips. Source contracts104pass. The next r47
changes only bounded observation coverage, not current authority behaviour.

R47captured the exact D1dispatch850/job849/source840 pair. CurrentReason
ContextRejected/ContextReasonSemanticChanged with actual prefix consistent.
Source/current fields differ only in normal fresh decision/observation/peer
generations and fingerprints, plus dynamic_obstacle_side_sign:+1 versus0.
The original worker proposal also declares side0. This is now same-frame causal
evidence, not the earlier cross-frame hypothesis. R47is a failed availability
run stopped after the diagnostic target was saved; not race acceptance.

Before production repair, r295 compares unchanged rejected current proposal
with an explicitly selected current disjunct (new sealed current fingerprint)
and the exact same immutable source/programme/world/history/full physical proof.
No equality rule is weakened. Selection is an ordinary Cruise/Follow avoidance
choice; opposite-side mission adoption is a different still-open contract.
Rollback for the next repair is89bda5bb.

R295 reproduced the original same-frame SemanticChanged and accepted the
explicit selected current disjunct with a6.359ms complete current physical
proof; source/programme unchanged, outside-deadline send rejected. Its guard at the
original nominal instant rejects; the physical steering step still has to fit
the actual predecessor-to-publication time. R298examines this separately.

The selected producer repair is `select_new_source_context`: Cruise/Follow's
unresolved dynamic side0 can choose an explicitly solved -1/+1 candidate only
if every other new-source semantic/model/schema/target/geometry field agrees.
Fixed sides, mission execution side and other intents gain no such permission.
The raw proposal and source remain immutable; the new selected current context
is resealed. The node performs this choice before its unchanged strict
dispatcher, and filters source candidates with the same compatibility rule
before worker evaluation, preventing an incompatible first source from hiding
later candidates. Existing actual published-remainder context is untouched.
No tolerance, expiry, physical proof or normal authority bypass is introduced.

R296passes140native tests/12suites. R297uses the production selection helper
and reaches the same independent current proof. R298retains the original
nominal-time slew rejection, accepts sampled offsets+1..+25ms in the original
25mswindow, and verifies a diagnostic actual ledger send/post-send identity at
the first accepted offset. Original deadline+1ns remains rejected. Fresh.now
is7.929999822, nominal7.939999823, predecessor actual7.914999823; physical
steering change0.02357361624rad. These are hypothetical dispatch clocks in a
saved replay, not live publication measurements. Full build/test and r48will
measure actual timing and sustained Ready/Start control.

Final local validation: buildr88passes26packages; testsr76passes2532records/
64groups with zero errors/failures/skips. Source contracts104pass. Required
actual runtime acceptance is next in dev2-r48. No moving completion is claimed.
