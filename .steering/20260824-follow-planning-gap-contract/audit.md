# Audit

## Hypotheses

### H1: target and ego progress use different coordinate origins

- Support considered: measured target gap is around 10--14 m while rejected
  retained minimum gap is around 2 m.
- Refutation of primary-cause status: failures overwhelmingly occur at terminal
  stage 19, not the current-state check.
- Residual finding: `front_distance` is ego-relative but was copied directly
  into a target tube documented as MPCC-origin-relative, omitting initial
  `e_lag`. This is a smaller real contributor and is repaired in the same
  coordinate contract.
- Confidence: refuted as primary cause; confirmed as contributor.

### H2: async compute age advances the cursor too far

- Support considered: a retained plan is evaluated after worker completion.
- Refutation: worker result age is normally 0.020--0.035 s, while rejected
  stages are terminal; only two jobs were replaced in more than 7,900
  submissions in the inspected run.
- Confidence: refuted as the primary cause.

### H3: nominal and physical Follow gaps are conflated

- Support: nominal gap is 4.0 m and physical hard gap is 2.05 m. Code uses the
  nominal gap only for reference but the hard gap for QP upper bounds. Terminal
  reward may therefore drive the plan to 2.05 m. Current-world updates reject
  around 1.94 m almost exclusively at stage 19.
- Refutation: demonstrate fresh solved terminal gaps retain material reserve
  above 2.05 m despite hard-bound QP wiring.
- Confidence: high; code and dynamic evidence agree.

## Classification

- Root cause: planning feasibility and physical failure boundary share the
  same Follow gap semantic.
- Contributors: target progress omitted the initial Frenet-lag origin offset;
  asynchronous production makes each valid current-world
  rejection visible as an immediate emergency-authority cycle.
- Mask: logs label the symptom `stage-gap-violation` without exposing that the
  fresh plan was intentionally allowed to consume nominal reserve.
- Detection gap: the contract has no separately logged planning gap.

## Implementation audit

- `FollowLongitudinalContract` now carries the current ego-relative target gap,
  the MPCC-origin ego offset, `planning_gap_m`, and `hard_gap_m` as separate
  typed values.
- The target tube is constructed once in the MPCC progress frame. The current
  ego-relative gap remains separately sealed and fingerprinted for the
  current-state physical check.
- The QP encodes Follow separation once, using
  `theta + e_lag <= target_progress - planning_gap_m`. The former duplicate
  theta-only target upper bound was removed; generic progress bounds remain.
- Fresh and retained physical certificates still use `hard_gap_m`. No proof
  tolerance, wall margin, solver setting, fallback, retry, timeout, lease or
  YAML value changed.
- Extended-problem construction verifies that the contract origin offset is
  identical to the five-state initial lag. A coordinate mismatch fails closed.
- Failure-first source enforcement failed before the repair because the
  planning-gap identity was absent, then passed after the production wiring.

## Dynamic audit

Compared the same Domain 1 Follow metrics between:

- baseline: `output/20260824-051821`;
- repaired: `output/20260824-055552`.

Results:

| Metric | Baseline | Repaired |
|---|---:|---:|
| retained `stage-gap-violation` trace lines | 611 | 4 |
| `canonical-follow-emergency` trace lines | 651 | 17 |
| dominant rejected stage | 19 (600/611) | 7 (4/4) |
| accepted retained minimum gap | terminal-bound behavior | 3.655 m minimum |
| configured physical hard gap | 2.05 m | 2.05 m |

The systematic terminal failure is gone. Accepted retained plans keep a
minimum observed reserve of 1.605 m above the unchanged hard boundary.

The four remaining stage-gap lines are two two-cycle episodes in which the
current target speed changes from approximately 4 m/s to exactly 0 m/s. They
reject at stage 7 with 1.57--1.79 m and recover on the next coherent target
observation. This is not the repaired terminal-bound defect; it is recorded as
a separate target-kinematics continuity issue and is not hidden by this Slice.

The run also entered Overtake three times. Two episodes later failed executed
wall-path proof and entered Recovery. That is downstream Overtake evidence,
not a reason to reopen the Follow gap contract in this Slice.
