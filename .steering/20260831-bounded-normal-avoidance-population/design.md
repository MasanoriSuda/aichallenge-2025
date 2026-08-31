# Design: bounded normal-avoidance population

## Before

`build_normal_avoidance_candidates()` returned exactly two members: one direct
candidate per side.  The branch coordinator then selected one pointer per side
and evaluated one SQP per branch.  In contrast, active Overtake already had a
bounded same-side population and stopped at the first fully certified member.

## After

Each normal side receives a small stateless population:

1. direct complete side disjunction;
2. measured-steering-reachable smooth side schedule;
3. midpoint schedule when distinct;
4. latest admissible side schedule ending at the immutable prediction
   boundary when distinct.

The latter candidates are not phases or persisted Mission paths.  They are
fresh references and disjunction schedules sealed to the current observation.
On every new epoch the population is rebuilt.

`evaluate_rate_resolved_normal_branch()` consumes one side's vector in anytime
order, shares only that branch's private solver context, and returns the first
complete certificate.  If none succeeds, it returns the strongest rejection
evidence.  The existing cross-side coordinator remains the only publisher
join.

## Why this is structural

The successful frozen C candidate delayed full lateral commitment while
decelerating, rather than forcing an immediately infeasible side row.  The
repair represents that missing topology family.  It does not weaken any hard
constraint and it leaves certified Stop as the fallback when the complete
bounded family is infeasible.
