# Recovery steering population live validation

2026-09-13. Runtime226e4f84, build138/package125, original DLL single-r23/120hostsec.
4260callbacks, maximum23.498078ms,522normal sends, no pre-send refusals or actual
post-send deadline violations. Last1159/job1144/source889/index12, nominal17.444999625,
before/after17.459999609, deadline17.469999625. Actual selected Rejoins983/source563,
1001/600,1118/837,1133/867; no Rejoin completion. Public max1.051338196mps at10.149999773.
Clock200Hz,velocity/steering28.571,IMU20,pose50,commands36.141. GameStart observed;
vehicleStart/laps absent. Original source/binaries/protected artifacts restored/verified,
runtime stopped. These are same-run observations, not a causal r22/r23 performance comparison.

All12committed Recovery maneuvers are Forward. New small-steering reverse execution
is unobserved; do not mark it passed. No Unknown-direction event was recorded;
R774 is correctly not-observed. Current source generation continues but later rejects
terminal Stop course geometry. Its diagnostic valid/feasible fields alone do not
prove preferred lateral containment or a usable exact physical interval.

The saved Rejoin candidate537uses preparation policy2 and rejects wall;538original
rejects refinement solve. Later saved656/743 also reject. Next replay these original
inputs and distinguish current contact, physical trajectory rejection, source creation
and absent late Stop envelope inputs. Preserve all wall/course/model/clock constraints.
Earlier moving rear hazard, physical observation/current velocity/r80actualsend and
allM4–M6obligations remain open. [Sealed evidence](recovery-steering-live-evidence.json).
