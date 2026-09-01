# Requirements

## Objective

Determine whether the successful speed-committed teacher correction is better
represented by a static spatial model or a causal temporal model before any
new runtime checkpoint is trained or promoted.

## Evidence split

- train: independently executed seed 2034;
- validation: independently executed seed 2033;
- both runs must pass Finish, penalty and stall gates;
- no sequence identity may appear in both splits.

## Constraints

- keep production v11 and launch authority frozen;
- preserve exact teacher, checkpoint, run and speed-sync provenance;
- retain every scan from each stateful teacher rollout;
- permit immutable train and validation sources to live in separate roots;
- reject an aggregate that lacks either split;
- run diagnostic representation probes before controller training;
- do not use validation metrics to relabel or train seed 2033.

## Definition of Done

- seed 2034 is strictly admitted and extracted as train;
- seed 2033 remains val-only;
- one recurrent dataset manifest binds both immutable source roots;
- raw/recurrent arrays and certificates validate end to end;
- static and temporal representations are compared on held-out seed 2033;
- the next model family is selected from evidence, not assumed in advance.
