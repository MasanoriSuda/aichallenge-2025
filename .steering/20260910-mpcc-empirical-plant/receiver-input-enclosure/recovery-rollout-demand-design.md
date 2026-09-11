# Recovery rollout demand audit, baseline808830ac

2026-09-12JST. M1–M6authorized; no routine confirmation, local commits only.
No production change yet. Do not claim this repairs R51's first current-proof cost.

R53diagnostic startup order reaches moving D1decision218/job217/source187 at0.11m/s.
Original nominal9.304999799, entry9.309999791, deadline9.329999799; before9.334999791.
Current proof13.334507mswall/11.125521msCPU. Callback21.402mswall/14.428141msCPU:
MPC14.204ms, Recovery5.066ms, publish/failsafe2.068ms. Proof ends atROS9.314999791;
Recovery ends9.334999791. In the same run, simulator frame551publishes three5ms
clock ticks in2.817mswall; frame552publishes four ticks in2.777mswall about16.7ms
later. Receiver set() follows them. Source clustering is observed; transport can
still contribute additional delay. Do not slow/suppress clock, alter original
appointments/windows/rates, or infer25mswall guarantees25msROS.

Next bounded audit: node evaluate_stuck_recovery calls evaluate_recovery_safety
with evaluate_rollout=true whenever recovery_safety_evaluation_required says the
slow/forward-intent normal state needs current wall evidence. The callee already
separates current footprint/contact/wall evidence from expensive forward/reverse
rollouts. Core RecoverySupervisor::update_normal consumes the detector verdict:
Normal can remain Normal or enter SuspectStuck with normal/hold; it does not
command a reverse/forward maneuver. This suggests a distinct demand predicate,
not dropping current safety evidence or caching a prior rollout.

Before any edit:
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
