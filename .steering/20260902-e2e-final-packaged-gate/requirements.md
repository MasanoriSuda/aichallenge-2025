# Requirements

## Objective

Evaluate the frozen packaged E2E production identity in the four-vehicle,
six-lap final reference world before any further model or parameter change.

## Frozen identity

- baseline commit: `2d0680eb`
- control mode: `fixed_lidar_brake`
- acceleration: `0.8 m/s2`
- maximum forward speed: `4.6 m/s`
- recurrent authority: disabled
- no runtime parameter or checkpoint override

## Constraints

- Do not modify production authority from a failed run.
- Use AWSIM result JSON, every domain bag and startup provenance.
- Classify timeout pace, contact/wall penalty, post-start stall, stale input and
  inference error separately.
- A clean but incomplete run is not a competition pass.
- No training artifact may be admitted from a failed or ambiguous domain.
