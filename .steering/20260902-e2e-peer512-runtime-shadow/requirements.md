# Requirements

## Objective

Qualify the peer-512 recurrent checkpoint for runtime shadow execution without
changing the published steering command.

## Constraints

- Convert the immutable PyTorch checkpoint with the existing strict converter.
- Re-verify embedded raw and production-spatial identities at runtime.
- Keep recurrent authority disabled and production defaults unchanged.
- Require three-lap Finish, zero penalties, zero stalls, at least 99% recurrent
  coverage, zero inference errors and zero unexpected hidden-state resets.
- Record callback/inference timing; reject the candidate if diagnostic load
  perturbs production execution.
- A passing shadow run grants no authority and does not package the artifact.

## Definition of Done

- NumPy conversion parity and artifact identity are recorded.
- The deterministic single-kart gate is executed and strictly analyzed.
- The candidate is admitted or rejected for a later interaction-shadow Slice.
