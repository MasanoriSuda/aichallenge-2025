# Design

## Hypotheses

### H1: the five-state solver is not the only source of authority churn

Supported by successful five-state cycles still ending as `legacy-normal-bypass`. The primal is
converted to a three-state layout before actuation and loses canonical plan identity.

### H2: reentry intentionally creates formulation switches

After an extended failure, a later successful solve is withheld for several cycles while the
three-state formulation remains production authority. Replay logs contain both circuit-open and
`requalifying extended solver 1/3` fallbacks.

### H3: an exact fresh canonical chain is already constructible

Track/Cruise and Follow use the same five-state primal normalization and canonical plan adapter.
Overtake already carries stage geometry, lateral corridor bounds and physical wall checks. The
missing part is wiring the exact artifact chain before legacy conversion.

## Shadow flow

```text
existing five-state solve result
  -> lateral row contract
  -> certified-bound normalization
  -> exact actuation proposal
  -> exact [ey, elag, epsi, v, theta] trajectory
  -> swept physical wall certificate
  -> CertifiedMpccSolution
  -> immutable CanonicalExecutionPlan
  -> exact cursor and FreshCertified authority
  -> canonical command and world prediction
  -> telemetry only
```

The current output path remains unchanged. No retained candidate is invented in this Slice because
Overtake requires current target/corridor revalidation, not the empty-world Track/Cruise proof or
the longitudinal-gap Follow proof.

## Promotion boundary

Gate A proves fresh exact execution. A later authority Slice must add current-world target/corridor
proof for a retained plan and atomically replace:

```text
five-state -> three-state -> legacy
```

with:

```text
fresh certified five-state
-> retained current-world-certified five-state
-> Emergency Stop
```
