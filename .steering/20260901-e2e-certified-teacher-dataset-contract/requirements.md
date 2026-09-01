# Requirements

## Objective

Bind a successful, actually executed teacher rollout to every dataset derived
from it.  Counterfactual labels and labels whose outcome is unknown must not be
mistaken for demonstrated successful actions.

## Constraints

- production v11, the frozen base checkpoint and runtime launch defaults remain
  unchanged;
- admission is run-level and fail-closed;
- the source run, domain, control mode, checkpoint and result artifacts must
  match the competition analysis exactly;
- the result, motion and competition-analysis hashes become part of the
  immutable dataset identity;
- historical datasets remain readable, but a strict builder can reject sources
  without an `executed_teacher_success` certificate;
- generated datasets and run output are not committed.

## Definition of Done

- one shared admission implementation owns successful-run validation;
- teacher relabeling can require an executed-success certificate;
- recurrent derivation preserves the certificate and offers fail-closed input
  admission;
- negative contract tests cover run, domain, mode, checkpoint and artifact
  mismatches;
- seed 2031 is extracted under the strict contract and its metadata is audited.
