# E2E DAgger v3 shadow acceptance requirements

## Objective

Evaluate DAgger v3 in the runtime callback without granting it steering
authority.  Confirm that the held-out transition improvement survives the ROS
2/AWSIM execution path before any limited-authority A/B.

## Frozen production contract

- candidate3 remains the only learned steering owner;
- runtime mode remains `fixed_lidar_brake`;
- spatial authority remains disabled;
- the v3 checkpoint is an explicit experiment input, not a default;
- no steering bound, safety distance or launch default may change.

## Acceptance

- deterministic `e2e-single` finishes 3/3 laps;
- penalty and stall counts are zero;
- shadow coverage is at least 99%;
- shadow errors are zero;
- minimum scan rate is at least 19 Hz;
- production checkpoint and mode admission still pass;
- runtime output remains production candidate3, not v3.

Passing this gate proves runtime availability only.  It does not prove that v3
improves closed-loop steering.
