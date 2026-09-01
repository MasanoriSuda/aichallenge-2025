# Requirements

## Objective

Make the admitted projected-conv5 recurrent candidate executable in the
NumPy-only participant runtime as a diagnostic shadow without changing the
published steering authority.

## Constraints

- Do not add PyTorch to the participant package or evaluation image.
- Do not change `/control/command/control_cmd` ownership or production output.
- Reject partial, non-finite, wrong-shape or wrong-hash recurrent artifacts.
- Verify that the recurrent artifact embeds the exact production raw and v11
  spatial baselines before enabling shadow inference.
- Preserve the 0.02 rad raw-correction deadband selected before seed 2035.
- Reset hidden state across sensor-stale, missing-speed and inference-error
  boundaries; temporal state must not bridge discontinuities.
- Keep the shadow opt-in.  No packaged recurrent default and no authority flag
  are introduced in this slice.

## Acceptance

- NumPy single-step recurrence matches PyTorch sequence inference within a
  declared numerical tolerance.
- Conversion is deterministic and records immutable source/output hashes.
- Legacy production output is bit-identical with shadow disabled and with an
  admitted shadow enabled.
- Invalid embedded identities and invalid temporal inputs fail closed for the
  shadow without replacing a valid production command.
- Existing participant topic and launch contracts remain unchanged except for
  additive optional shadow arguments.
- A three-lap closed-loop run finishes with zero penalties, zero recurrent
  skips/errors/resets and no material production timing or lap-time regression.
