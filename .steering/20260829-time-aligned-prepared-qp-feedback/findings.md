# Findings: time-aligned prepared-QP feedback

## Root cause isolated by the preceding experiment

The rejected feedback connector did not fail because the latest observation
was intrinsically infeasible.  It combined latest x0 and a later timestamp with
future state boxes, refinement rows and affine dynamics owned by the old
control origin.  The resulting QP was a mixed-origin problem.

## This Slice

The observation-only builder now transports the final refined QP to its exact
unconsumed absolute-time suffix.  It:

- consumes state, input, wall and obstacle stages with one clock;
- shortens the active first stage;
- rebuilds the steering prefix from latest physical steering;
- retains and renumbers every surviving refinement row;
- rejects malformed refinement provenance before filtering elapsed rows; and
- relinearizes all surviving dynamics around a suffix-owned tangent.

The deterministic counterexample changes classification from solve rejection
to solved without changing tolerance, iteration limit, clearance or authority.
This supports a scheduling/lifecycle formulation defect rather than physical
infeasibility for that case.

## What is not proved

The three-stage fixture is intentionally deterministic and too small to prove
live timing.  The captured 20-stage failure snapshots contain both the semantic
request and final assembly request, but the current public replay loader exposes
only the semantic source and assembled matrix QP.  The next observation Slice
must expose the recorded assembly request and evaluate this exact transform on
frozen live snapshots before any runtime shadow or production promotion.
