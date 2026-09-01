# Requirements

## Objective

Determine whether the frozen spatial representation can simultaneously retain
corrective direction and classify independent production-normal states as
neutral after train-only normal anchors are admitted.

## Constraints

- diagnostic classification only; no checkpoint or runtime path
- same teacher train/validation and peer-d3 split
- normal train labels are forced neutral; stored commands are ignored
- independent normal validation remains unseen
- compare with unchanged weighted classifier settings for three seeds

## Exit classification

- corrective and normal classification both pass: continuous head/loss defect
- normal passes but corrective fails: representation/data overlap
- corrective passes but normal fails: missing normal support or representation gap
- both fail: input/data contract is inadequate
