# Recovery rollout demand, baseline e1ce2571

2026-09-12JST. M1–M6authorized; no routine confirmation, local commits only.
The bounded production change below preserves current detector evidence.
Do not claim this repairs R51's first current-proof cost.

R53diagnostic startup order reaches moving D1decision218/job217/source187 at0.11m/s.
Original nominal9.304999799, entry9.309999791, deadline9.329999799; before9.334999791.
Current proof13.334507mswall/11.125521msCPU. Callback21.402mswall/14.428141msCPU:
MPC14.204ms, Recovery5.066ms, publish/failsafe2.068ms. Proof ends atROS9.314999791;
Recovery ends9.334999791. In the same run, simulator frame551publishes three5ms
clock ticks in2.817mswall; frame552publishes four ticks in2.777mswall about16.7ms
later. Receiver set() follows them. Source clustering is observed; transport can
still contribute additional delay. Do not slow/suppress clock, alter original
appointments/windows/rates, or infer25mswall guarantees25msROS.

Causal audit: node evaluate_stuck_recovery calls evaluate_recovery_safety
with evaluate_rollout=true whenever recovery_safety_evaluation_required says the
slow/forward-intent normal state needs current wall evidence. The callee already
separates current footprint/contact/wall evidence from expensive forward/reverse
rollouts. Core RecoverySupervisor::update_normal consumes the detector verdict:
Normal can remain Normal or enter SuspectStuck with normal/hold; it does not
command a reverse/forward maneuver. This suggests a distinct demand predicate,
not dropping current safety evidence or caching a prior rollout.

Audit requirements (completed for this slice):
- Audit every node consumer of safety fields around57639–58580 and the supervisor
  pre-switch validation. Check entry-side effects and coordinated/overtake handoff.
- Narrow candidate only to Normal with healthy normal authority, no dynamic lateral
  execution, rearm guard, fallback or invalid inputs. Every existing episode,
  fault retry, explicit dynamic owner and all maneuver/gear/rejoin paths keep fresh
  full evaluation. Current wall/contact observations and detector clocks remain.
- Establish a meaningful regression over Normal→SuspectStuck→clearance/maneuver
  transitions, including changed wall/peer/gear evidence. No action may use an
  uncomputed field as an affirmative permission. Do not double-update the detector.
- If any Normal-entry consumer needs full rollout now, retain full evaluation for
  that case or reject this optimization. Preserve trace semantics and downstream
  current-wall evidence. No thresholds, hysteresis, timeout, model or bounds changes.

R51first movingD1856had only0.014msRecovery and18.33ms current proof. Therefore
this is a separate R53timing contributor and cannot close the overall proof-cost
problem by itself. Source full-rest proof and independent current full-rest proof
remain necessary. R193/r194old point-partition reuse remains rejected.

A later structural alternative, not implemented or validated, is a separately
certified initial state/time domain built in the worker: dispatcher membership
would refer to that independent starting domain, never marginal containment in
an old reachable over-approximation. It would require full body/corner/rest/world
proof for every start state/time and original input history/window, distinct hash,
exact current membership and unchanged prefix/context/final guards. Merely widening
measurement tolerances, declaring covariance a hard error bound, translating old
certificates or retiming prior programmes is not this theorem. Feasibility and
cost of such a domain are Unknown; do not promote from this design note.

Implemented 2026-09-12: a pure demand policy returns None, CurrentFootprint or
FullRollout. None preserves the old no-evidence decision. CurrentFootprint requires
the existing joined certified normal command, supervisor Normal, finite speeds,
no fallback/rearm/dynamic lateral execution, no coordinated stop, no validated
overtake handoff and no pending drive handoff. All other requested cases keep
FullRollout; fault retries and active maneuver/rejoin evaluators remain unchanged.
The node uses the evaluator's existing evaluate_rollout boundary. Current wall,
footprint, contact and detector times still run; no prior clearance is reused.

Normal update can only remain Normal or enter SuspectStuck. The existing
recovery_candidate_commit_allowed rejects Normal, SuspectStuck and waiting states;
Normal entry cannot commit a maneuver from uncomputed fields. Pre-switch finite
checks do not require affirmative maneuver fields in Normal. Coordinated/overtake
consumers explicitly retain full evaluation. The next active Recovery tick uses
fresh full wall/peer/gear evidence before any maneuver selection. No authority,
physical model, source identity, bound, margin, timing or clock setting changes.
The obsolete unconditional rollout request is removed only for this proven case.
Existing current-wall trace remains; telemetry separately counts deferred rollouts.

R314 applies the old unconditional-full policy to the new tests and intentionally
fails both demand expectations; it is a failing specification regression, not a
claim that an old production test failed. R315 compiles the actual changed policy
and entire core suite: 147 tests in 31 suites pass. Paired real-core executions
compare detector verdict/duration and supervisor state/action across 180 ticks
from Normal into clearance with freshly blocked reverse peer evidence; no gear
request or creep is authorized. Missing certified authority, active/rearm/fallback
contexts and invalid state/speed retain full evaluation. Source contracts: 105 pass.
Build r92: 26 packages pass. Package tests r80: 2547 records in 65 groups,
zero errors/failures/skips. Standard uninstrumented dev2-r54 remains pending.
[Sealed evidence](recovery-rollout-demand-evidence.json).
Rollback is e1ce2571 plus reversal of this slice only, preserving unrelated changes.

Standard dev2-r54 on committed dbbe5fe6 confirms deferred work: D1fully-deferred
567cycles weighted Recovery0.067268ms, D2324cycles0.070790ms; total deferred
916/439. This includes startup and Ready and is not a paired speedup benchmark.
R54fails D1post1157before shutdown; current proof16.208mswall/11.252msCPU,
Recovery0.073ms. Original before16.374999633 passes deadline16.374999644;
after16.379999633 fails. R316 exact physical proof and before/after failure match.
Full logs add D1post1584during teardown and D2pre1466just after shutdown began;
preserve them separately. No laps or integrated acceptance.
[Next independent-domain comparison](starting-domain-design.md).
