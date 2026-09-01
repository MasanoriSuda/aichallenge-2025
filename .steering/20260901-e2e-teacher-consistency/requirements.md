# Requirements

## Objective

Determine whether the unresolved normal/material trade-off is caused by
inconsistent target semantics rather than another model architecture defect.

## Questions

1. Does the current precontact teacher reproduce every stored recurrent
   teacher label from the exact same physical scan and frozen base checkpoint?
2. Does that same deterministic teacher request material corrections on the
   successful production-normal scans currently declared as zero residual?

## Constraints

- no training, model, threshold or runtime changes
- audit complete train and validation splits
- keep source identity and decision reasons per sequence
- compare stored/current labels at 1e-6 rad
