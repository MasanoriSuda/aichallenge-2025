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
