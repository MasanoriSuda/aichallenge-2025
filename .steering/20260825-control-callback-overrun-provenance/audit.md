# Audit

## Observed phenomenon

Two independent dev2 runs each had exactly one control callback over the 25 ms
budget. The periodic aggregate retained only the maximum total duration. Other
logs use separate windows and cannot be joined to the exact offending decision.

## Root observation defect

The callback already measures total elapsed time but does not preserve timing
ownership or decision identity for the maximum/overrun cycle. Therefore a
production solve, wall proof, Recovery scan and publication work remain
indistinguishable. Optimizing any one of them now would be another speculative
patch.

## Authorized repair

Add exact, observation-only region timing to the existing callback reporter.
No runtime owner is moved and no threshold is changed.

## Dynamic conclusion

`output/20260825-023027/d2/autoware.log` exercised the failure at decision
2096:

```text
total=26.098ms / budget=25.000ms
pre_mpc=0.055ms
mpc=20.786ms
post_mpc=0.020ms
recovery=5.046ms
publish=0.189ms
unattributed=0.002ms
checkpoint=complete
```

This falsifies post-MPCC wall/authority work and publication as the dominant
owners. The callback exceeded its period because a long but still successful
production MPCC cycle and synchronous Recovery evaluation accumulated in an
ordinary Cruise cycle. The nearby Track/Cruise aggregate remained fully
solved/certified and recorded a 20.362 ms maximum canonical total, consistent
with the exact `mpc_ms` measurement. The asynchronous rate-resolved shadow was
not selected and is not the callback owner.

Code audit then found the upstream scheduling defect: while the Recovery
supervisor is `Normal`, `evaluate_stuck_recovery()` classifies nearby walls and
samples the current footprint before `StuckDetector` can reject a clearly
moving vehicle. That ordering can consume several milliseconds even when no
Recovery candidate can exist. This finding authorizes a separate Slice that
puts cheap eligibility before expensive Recovery safety evaluation, while
preserving full evaluation for low-speed candidates and every active Recovery
state.
