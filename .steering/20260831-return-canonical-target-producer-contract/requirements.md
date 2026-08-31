# Requirements: Return canonical target producer contract

## Objective

Repair the first upstream invariant that prevents a prospective Return from
acquiring production authority in `output/20260831-093415/d1`.

Frozen evidence:

- last useful transition: `ShiftOut -> Pass`;
- repeated Return request begins around decision 1717;
- Return Gate-A build rejection:
  `canonical current-epoch target tube unavailable`;
- first downstream authority loss: decision 1839;
- immutable failure snapshot:
  `000000001839-4390e2814c72a13d-pass-side-negative-physical-proof-terminal-contingency-unavailable`.

## Root-cause gate

The stateless Return consumer requires the current-epoch target tube to prove
whether rejoin stays ahead of or behind the target. The canonical target-tube
producer must therefore support Return. No alternate predictor or retained
Mission geometry may be introduced.

## Constraints

- no Mission resume rule, lease, grace period, timeout or fallback;
- no solver tolerance, weight, wall margin or clearance change;
- no production authority graph change;
- keep stage-corridor target bounds exclusive to ShiftOut/Pass;
- Return may consume only the canonical current-world target tube;
- preserve the exact ReplayWorld identity and proof join.

## Definition of done

- producer and consumer intent contracts agree for Return;
- a focused test proves Return selects the current target tube, never the
  passing stage corridor;
- all package tests pass;
- a bounded dynamic run shows Return proposals proceed beyond the former
  `canonical current-epoch target tube unavailable` rejection;
- results and remaining independent failures are recorded.
