# Design

The current dataset has one numeric steering target per observation, but the
target has several materially different causal meanings.  This audit assigns
one of the following evidence classes without changing the stored target:

- `executed_teacher_success`: the exact teacher controlled the source run and
  the domain finished with zero penalties;
- `executed_teacher_failure`: the teacher controlled the source run but the
  domain did not pass;
- `successful_alternative_policy`: another policy passed, so that executed
  action is demonstrated safe but an offline teacher proposal is not the only
  valid action;
- `counterfactual_teacher_on_failure`: another policy failed and the teacher
  was only replayed offline;
- `outcome_unproven`: result-detail evidence is absent or invalid.

Only `executed_teacher_success` may be treated as a hard teacher demonstration.
Failure-derived teacher output remains useful as a proposal, but requires a
paired successor rollout or another intervention-necessity certificate before
becoming an exclusive target.  A certified successful base run demonstrates
that zero correction is valid along its visited trajectory; it does not prove
that every non-zero teacher alternative is unsafe.

The report therefore exposes both sequence count and sample count.  Long,
unproven recordings must not silently dominate merely because they contain more
frames.

## Result

The current corpus is not ready for exclusive action training:

| Corpus | Sequences | Samples | Outcome evidence |
|---|---:|---:|---|
| production normal | 3 | 17,747 | all certified successful alternative-policy runs |
| teacher | 3 | 6,884 | counterfactual teacher replay on certified failure |
| teacher | 2 | 12,431 | offline teacher proposal on a certified successful alternative policy |
| teacher | 10 | 34,393 | outcome unproven |

The normal corpus contains three demonstrated zero-action sequences.  The
teacher corpus contains zero `executed_teacher_success` sequences.  Its 8,492
material targets are therefore proposals, not outcome-certified exclusive
actions: 1,397 came from certified failures, 1,062 from another successful
policy and 6,033 from runs without strict result evidence.

The next data slice must collect an exact precontact-teacher run with immutable
result-detail evidence and admit it only after Finish, zero penalty and zero
stall.  If that teacher cannot pass, its heuristic output cannot remain the
primary hard target and must be replaced with a policy demonstrated in a
successful rollout or paired intervention evidence.
