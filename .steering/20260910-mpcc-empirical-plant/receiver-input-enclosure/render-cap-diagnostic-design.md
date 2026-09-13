# Separate the original Unity rendering cap from rendering workload

2026-09-13, control7ecd0b36/build142/package129. Original DLL and source fixed.
The earliest broken invariant is actual normal publication exceeding its
immutable original25ms ROS window in r26/2180 and r30/8440. R53 observed
3/4FixedUpdate clock ticks grouped per16.7ms frame. r27 removed graphics and,
as the sealed original RuntimePerfMetrics CIL now establishes, the60FPS cap.
This is a concrete confound, not evidence that rendering itself is repaired.

Bounded single-r31 changes only the supported Unity --target-fps 200 argument
relative to standard graphical r30.200 derives from the existing5ms physics
cadence; it is a diagnostic comparison, not rate tuning or production promotion.
Keep graphics/RViz, original startup/DLL, sensors, physics/clock timestamps,
controller/source/guards and six-lap600sim configuration. Use120hostsec bound
and stop on actual publication/launch failure. A positive cap preserves the
original vSyncCount=0 assignment; zero/off skips that assignment and would add
another variable. Preserve source/binary/DLL/config/run identity and protected
user artifacts. Do not rebuild or change source/docs during this run.

Measure same-run /clock source increments and recorder receipt gaps, normal
actual pre/raw/post timestamps, callback regions/CPU, source/proof route,
Recovery/Rejoin, Start/laps. Recorder receipt is not producer frame time or
controller reception. Changed paths/scene and unknown default random seed
limit paired causal attribution. A positive bounded run cannot establish a
future timing bound or whole race acceptance. No epoch shift, clock freeze,
margin, solver, timeout, grace, retry, fallback or publication guard relaxation.

Temporary current-interval observer is retained only to keep this single-factor
comparison clean, then removed with exact1263 boundary regression. No new
observer/normal authority is introduced. Diagnostic startup overlay is scoped
to run output; it leaves the default repository startup unchanged. Archive
accepted/rejected/inconclusive outcomes, then choose a supported next action.
Rollback is removal of this run overlay; no production code to revert.
[Failure evidence](full-course-current-audit.md).
