# Design

## Hypothesis under test

The five-state bicycle model optimizes a model-equivalent tire angle.  The AWSIM integration has a
separate calibrated actuator mapping:

```text
actuator steering request = model steering angle * steering_tire_angle_gain_var
```

Before canonical promotion this mapping was applied to legacy output.  The experiment asks whether
the same mapping is also required by the new five-state canonical closed loop.  Existing three-lap
evidence with exact canonical publication conflicts with that hypothesis, so only a reversible,
typed implementation followed by dynamic falsification is acceptable.

## Experimental implementation

Extend `CanonicalNormalCommand` with two distinct values:

```cpp
double model_steering_tire_angle_rad;
double actuator_steering_tire_angle_rad;
```

`build_canonical_normal_command()` receives the existing positive finite actuator gain and computes
the actuator request.  Both values are immutable after construction.  The controller continues to
use model steering for model/history bookkeeping, while `/control/command/control_cmd` receives the
stored actuator request exactly.

The publisher accepts an optional explicit canonical actuator angle:

- canonical normal: publish the command's stored actuator angle exactly;
- legacy normal, canonical Emergency and Recovery: retain the existing raw-angle times configured
  gain convention;
- invalid/non-finite inputs: fail closed.

This does not reintroduce legacy Track/Cruise authority.  It is retained only if dynamic evidence
shows that the calibrated closed loop is at least as stable as exact canonical publication.

## Dynamic decision

`output/20260823-072038` rejected the hypothesis.  With the 1.5 multiplier active, the first lap
became oscillatory through wp77--108.  At wp92 the command had already changed to positive while
measured curvature remained negative; at wp108 the command had changed negative while measured
curvature remained positive.  Speed then changed from about 8.03 m/s to 0.37 m/s before wall
contact at wp113.  The experimental source and tests are therefore removed rather than tuned.

## Failure-first tests

1. A 0.20 rad model angle with gain 1.5 produces an immutable 0.30 rad actuator request.
2. Invalid/zero calibration rejects command construction.
3. Model-actuation identity and actuator-publication identity are checked independently.
4. Canonical publication uses the supplied actuator request exactly and never multiplies it again.
5. Legacy publication still applies the configured gain.

## Rejected outcomes and alternatives

- Retain the typed multiplier after the dynamic regression.
- Re-enable the generic publisher multiplier only for canonical commands.
- Change the configured gain from 1.5.
- Add wall margin to hide the actuation mismatch.
- Add steering low-pass, retry, retained lease or legacy fallback.
- Treat Recovery tuning as a fix for the pre-Recovery collision.
