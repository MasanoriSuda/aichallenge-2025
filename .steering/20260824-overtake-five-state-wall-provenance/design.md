# Design

## Observed failure

In `output/20260824-055552/d1/autoware.log`, Overtake enters three times. The
first two entries are certified and committed, but later fail during ShiftOut:

- episode 1: entry at waypoint 158 with `required_wall=0.400 m`; live execution
  later reports `solution hard wall contact`;
- episode 2: entry around waypoint 170 with a frozen reference reporting about
  `1.35 m` wall clearance; after 13.77 m of ShiftOut, live execution reports a
  swept wall collision at path index 16 and enters Recovery.

The failure is not an entry-time missing certificate and not a configured wall
margin change. The certificate exists, is fresh and is accepted. The same
Mission later becomes physically infeasible under live execution validation.

## Root-cause hypothesis

The five-state extended MPCC solves
`(lateral, lag, heading, velocity, progress)`, but branch certification converts
the solution to the legacy three-state layout and then stores only lateral
samples and nominal path distances. Entry revalidation again creates a
lateral-only trajectory. Consequently:

- solved lag is discarded;
- solved heading is reconstructed from lateral differences;
- solved progress is replaced by nominal waypoint/path-distance sampling;
- the certificate cannot identify the exact solved pose sequence.

The live wall monitor evaluates a yawed footprint and swept connections, so the
certificate and the rejection do not prove the same artifact. A comment calls
the branch trajectory exact, but the schema makes that claim impossible.

## Hypotheses to test

1. **H1 — lossy certificate schema (high confidence):** identical lateral
   samples can be safe in the legacy projection and unsafe with the solved
   lag/heading/progress. Refute by proving those fields cannot alter the wall
   result.
2. **H2 — pointwise QP wall rows omit swept footprint constraints
   (contributor):** even an exact five-state solution can satisfy stage bounds
   and collide between stages. Refute by exact proof passing for every rejected
   live solve.
3. **H3 — progress-coupled linearization drift (possible contributor):** solved
   progress moves into geometry not represented by nominal stage bounds. Refute
   by a small solved/reference progress delta at the first rejected pose.
4. **H4 — reference clearance logging is stale (detection mask):** the displayed
   1.35 m is Mission/reference evidence, not the exact live solution reserve.

## Intended structural repair

Create one exact physical trajectory artifact from
`extract_extended_execution_trajectory` and use it before any legacy
conversion. Its certificate must preserve lateral, lag, heading, solved
progress and course-frame provenance. The same schema is then consumed by
entry revalidation and live execution validation.

Legacy conversion may remain temporarily as a command adapter, but it must not
own physical proof or Mission admission.

## Rejected alternatives

- Increase wall margin: changes tuning while leaving the semantic split.
- Add an entry cooldown or Recovery retry: treats the downstream symptom.
- Permit the live rejection and switch sides: preserves admission of an
  unidentifiable artifact.
- Disable swept footprint validation: weakens the only guard currently exposing
  the defect.
