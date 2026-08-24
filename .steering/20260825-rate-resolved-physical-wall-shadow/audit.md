# Audit

## Observation

The five-state production/shadow path first extracts exact lateral, lag,
heading, velocity and solved progress. It then reconstructs world poses from
course-frame knots, samples the full footprint at every state and sweeps from
the current measured pose between those states.

The six-state shadow already retains those pose fields plus steering, but its
runtime log ends with `physical=not-evaluated`. Its lateral QP boxes only prove
the numerical corridor rows and cannot detect footprint-wall contact between
stages.

## Root cause

The new solver-to-artifact boundary was completed before the
artifact-to-current-world physical-certificate boundary. Promoting the artifact
now would skip a safety invariant already required of the five-state owner.

## Chosen correction

Reuse the single established physical wall implementation through a typed
adapter. Do not add a second wall model, relax a margin or execute the artifact
in this Slice.
