# Track/Cruise authority promotion gate

## Baseline

- Branch: `develop_july`
- Baseline commit: `5bae30b`
- Preserve `aichallenge/result-summary.json`.

## Purpose

Freeze the evidence and decisions required before Track/Cruise final command ownership can move
from the legacy three-state path to canonical five-state MPCC.

## Completed preparation

- immutable problem/solution/physical-certificate identity;
- fresh and retained selector policy with no cross-formulation normal fallback;
- complete five-state/three-input plan store;
- direct certified-primal extraction without legacy flattening;
- Track/Cruise shadow store population;
- exact fresh cursor, current proof and selector admission;
- exact cursor-to-actuation extraction and shadow round-trip check;
- explicit current Track/Cruise supervisor intent and exact candidate-intent matching.

## Promotion blockers

1. Fresh dynamic evidence exists in `output/20260822-232351`; its certified chain is exact. Three
   non-executable results were rejected before certification and establish the need for retained
   same-formulation continuity, not a shared-solver admission change.
2. Retained execution has no world-frame current-pose/current-observation revalidation implementation.
3. Retained lateral bounds must be aligned by absolute progress; indexing old plan stages against
   current problem stages is not a valid proof.
4. The final publisher still consumes the legacy solution. Switching it is an explicit authority
   change and requires approval plus a rollback run.

## Do not do before the gate passes

- Do not index old lateral bounds by the retained cursor and call that current revalidation.
- Do not reuse the original solve's wall certificate as a current execution certificate.
- Do not use the legacy-converted vector as canonical retained actuation.
- Do not add another mode flag that permanently preserves both normal authorities.
