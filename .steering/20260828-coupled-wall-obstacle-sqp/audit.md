# Audit

## Classification before implementation

The frozen A pipeline fails because one SQP refinement orders two individually
valid constraint producers incorrectly. The available evidence classifies the
live failure as a single-SQP/refinement lifecycle defect, not as a parameter or
physical-clearance defect.

## Rejected non-fixes

- increasing OSQP iterations or tolerances;
- reducing wall or opponent clearance;
- retaining the preceding trajectory longer;
- adding another mission fallback;
- deleting the dynamic-obstacle row that reports the contradiction.

These would hide the fact that the producer has removed longitudinal freedom
before it asks the optimizer to stay behind the target.

## Implemented correction

`mpcc_rate_resolved_dynamic_obstacle::Request` now separates the wall-only
classification witness from an optional compatible constraint target. The
producer derives exactly the same disjunct from the witness but copies the
generated rows onto the target problem.

When wall and obstacle refinement are both active, the canonical solver now:

1. solves a wall-only witness;
2. applies its selected dynamic branch to the broad relinearized problem;
3. solves the dynamic-aware provisional problem;
4. rebuilds wall/footprint constraints around that trajectory;
5. solves the joint problem before exact publication proof.

Warm starts across different row sets rebuild state and dual values from the
receiving problem instead of transporting stale equality/row provenance.

## Verification

- `make autoware-build`: passed.
- Focused dynamic-obstacle producer tests: 12/12 passed.
- Focused shadow tests, including coupled wall/obstacle refinement: passed.
- C++ package CTest: 35/35 passed.
- Python contract and architecture audit CTest: 12/12 passed after supplying
  the tools' documented `PYTHONPATH`.
- Dynamic Gate: `output/20260828-032723`.
- The frozen `dynamic-obstacle-effective-progress/stage=16` contradiction did
  not recur.
- Episode 3 completed `ShiftOut -> Pass -> Return -> Idle` without contact
  escape. Pass lasted about 1.14 s and the complete episode 12.05 s.

## Next isolated defects

The same Gate exposed two independent failures before the successful episode:

- wall-refined QPs can reach maximum iterations before dynamic refinement;
- one ShiftOut can consume the persistent same-target Mission budget before
  reaching Pass.

These are not recurrences of the coupled-order defect. They require frozen
snapshot analysis in subsequent structural Slices; no solver or Mission
parameter is changed here.
