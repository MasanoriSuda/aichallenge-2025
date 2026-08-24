# Audit

## Observed phenomenon

One ordinary Cruise callback exceeded its immutable 25 ms period. Exact timing
assigned 20.786 ms to production MPCC and 5.046 ms to Recovery evaluation.

## Symptom versus cause

The callback overrun is the downstream symptom. The upstream cause is that the
Recovery adapter computes map-derived safety evidence before establishing
whether the detector can consider the current moving vehicle a candidate.
`StuckDetector` then rejects it as `VehicleMoving`, making the earlier map work
irrelevant to the output.

## Existing patch interaction

Recovery has accumulated wall, collision, solver, coordinated-stop and AWSIM
handoff policies. Those policies remain required once a vehicle is slow enough
to be a candidate or a Recovery episode is active. The repair therefore does
not delete or bypass any of them; it changes only their scheduling boundary.

## Authorized repair

Make Recovery safety eligibility a typed pure contract and apply it once to
all map safety acquisition. Do not optimize MPCC or tune Recovery parameters in
the same Slice.

## Dynamic conclusion

The committed-source dev2 run confirmed the causal hypothesis. Once normal
motion began, both domains produced skip-only windows and Recovery time fell
from millisecond-scale map work to roughly 0.01--0.02 ms while the detector
continued to report `Moving/vehicle_moving`. Windows before race start and
other explicitly eligible contexts retained full safety work. No callback
overrun occurred.

The remaining 22.679 ms maximum is no longer a Recovery-scheduling defect: its
Recovery region was only 0.013 ms. Production MPCC still owns the largest
runtime tail and should be assessed separately as part of the rate-resolved
authority migration, not patched by changing Recovery or control cadence.
