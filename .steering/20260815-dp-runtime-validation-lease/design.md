# Design

## Policy

`FrenetDpPassAuthorityResolution` accepts authority from either:

1. a newly generated same-target/same-side DP path within the existing
   optimizer warm-start age; or
2. the same loaded path whose executed horizon passed runtime physical
   revalidation within a short lease.

The runtime lease defaults to 0.20 s. At the 40 Hz controller rate this spans
several scheduling cycles but cannot hide a sustained validation outage.

## Runtime revalidation

After the DP reference and live receding horizon are evaluated, renew the
runtime lease only when all of the following hold:

- phase is `Pass` and the DP execution reference covers the current horizon;
- final execution horizon is wall/lateral-acceleration feasible;
- no target-bound hard infeasibility or fallback path replaced it;
- target identity and course progress remain continuous;
- current physical bodies are separated, and either the physical prediction
  sweep is separated or the live target-bound horizon itself is feasible;
- no wall hard fault, emergency risk, solver recovery or forbidden waypoint.

The timestamp belongs to the currently loaded DP path. Freezing or refreshing
a path resets it before the new path is first validated.

## Authority ordering

The Pass horizon extension decision occurs before the current cycle's horizon
evaluation. It therefore consumes the previous cycle's runtime-validation
timestamp. The current cycle renews the timestamp only after successful
execution validation, forming a bounded one-cycle feedback path without
reordering the controller.

## Logging

Authority transition logs report whether ownership came from a fresh optimizer
source or runtime revalidation, plus both ages.
