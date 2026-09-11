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
