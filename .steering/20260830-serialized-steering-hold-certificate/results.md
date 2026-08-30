# Results

## Root cause

The physical continuation and terminal Stop builders initialized their state
from the serialized tire angle, then immediately integrated the SQP steering
rate over the first publisher interval. ROS publishes a tire angle, not that
internal steering rate. The certificate was therefore one rate step ahead of
the command it authorized.

This is a model/certificate mismatch. It is not a solver tolerance, wall
clearance or Mission lifetime problem.

## Change

- Hold the serialized tire angle over the first publication interval in both
  normal continuation and terminal Stop construction.
- Keep serialized acceleration active over that interval.
- Resume the SQP steering-rate sequence only after the boundary.
- Derive stage-end steering from the exact nonlinear rollout state.

No production authority, Mission lifecycle, fallback or parameter changed.

## Static verification

- `make autoware-build`: 25 packages passed.
- physical adapter tests: 20/20 passed.
- retained publication tests: 51/51 passed.
- Stop successor observation tests: 5/5 passed.
- source contract tests: 85/85 passed.

The tests explicitly require zero steering rate and unchanged tire angle over
the first publisher interval, followed by resumption of future SQP rates.

## Dynamic verification

Baseline: `output/20260830-143906`

- first 40 Cruise joins: command-origin steering error mean/max
  `0.008772/0.017856 rad`;
- later windows retained the discrete one-rate-step pattern up to
  `0.025173 rad`.

Changed run: `output/20260830-144538`

- first 42 Cruise joins: `0.003638/0.007317 rad`;
- 10 ShiftOut + 1 Pass joins: `0.002661/0.007317 rad`;
- first 7 Pass joins: `0.000000/0.000000 rad`;
- next 41 Pass joins: `0.000446/0.003659 rad`;
- following 41 Pass joins: `0.000368/0.003659 rad`.

The command-control-origin and previous-published errors remain identical, as
expected from the serialized command boundary. Current-time physical and
response-origin values are not the correct successor join and remain larger.

The run reached `Idle -> ShiftOut -> Pass`, so the correction was exercised by
the intended dynamic intents rather than Cruise only.

## Separate failure observed

The same run later stopped in Pass. This was not a wall-contact rejection in
the changed certificate. The current artifact became stale, retained
publication became unavailable, the normal authority selected emergency Stop,
and the stopped vehicle then entered stuck recovery. The episode ended with
`external recovery completed`.

That authority/lifecycle failure is deliberately not patched in this Slice.
It is the next root-cause subject and must not be hidden by weakening this
certificate.

## Conclusion

The serialized steering hold removes the systematic one-rate-step proof error
and is accepted as a root-cause fix. A certified Stop suffix still needs a
separate timing/handoff audit before it can receive production authority.
