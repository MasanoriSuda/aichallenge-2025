# Pro review handoff

## Review objective

Review the current `develop_july` HEAD after the commit containing this Slice.
The long-term objective remains one canonical Race MPCC normal authority, with
SafetyBrake and physical Recovery as explicit external supervisors. Parameter
tuning remains frozen.

Please judge whether the latest Stop lateral repair preserves that architecture
and whether a better bounded formulation is required before proceeding.

## Failure and root-cause claim

The apparent failure was an interrupted ShiftOut that could not reconnect
after SafetyBrake. Audit of `output/20260827-214537/d1/autoware.log` found the
upstream cause: Stop entered at 5.15 m/s, froze steering at `-0.159 rad` while
course curvature changed, and drove into a wall before the reconnect failed.

The repair keeps Stop as the sole Emergency wire authority but, while moving,
rate-limits steering toward current reference-path feedback. It neutralizes if
that target is unavailable and holds only at rest. It never publishes or marks
the latent normal shadow as executed.

## Evidence status

- Build: 25 packages passed.
- Tests: 47/47 targets; 1,938 tests; zero errors/failures/skips.
- Dynamic no-regression: `output/20260827-221458`, about two minutes of dev2,
  no wall contact and normal Cruise recovery.
- Dynamic acceptance: incomplete because no explicit SafetyBrake Stop occurred
  in that run.

## Questions

1. Is external Stop with path-following lateral control a coherent supervisor,
   or should Stop itself become a constrained MPCC intent before further
   production acceptance?
2. Is using the established spatial feedback plus lateral-acceleration envelope
   sufficient for a bounded emergency policy, or is a stopping-distance wall
   certificate required before publishing each Stop steering step?
3. Does the separation between the normal-only intent ledger and effective
   wire-authority ledger remain correct after this change?
4. Should the next Slice build a deterministic moving-Stop acceptance scenario,
   or return to the remaining Slice 6 legacy-code deletion first?
5. Are there relevant MPCC/emergency-path-following algorithms or open-source
   implementations worth comparing before adding more code?

Please distinguish architectural defects from dynamic evidence still missing;
do not recommend parameter tuning as a substitute for either.
