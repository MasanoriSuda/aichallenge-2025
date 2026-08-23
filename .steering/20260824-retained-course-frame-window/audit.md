# Audit

## Failure-first evidence

- Run: `output/20260824-043223`
- Overtake async producer: 157 completed, 155 exact physical canonical chains.
- Current-world ready/stored: 124/123.
- Rejections: course-frame unavailable 23, stage-corridor violation 9,
  corridor-horizon unavailable 1.
- Follow production independently logged intermittent
  `world proof rejected: course-frame-unavailable/invalid-plan` and emergency
  commands, demonstrating that the defect belongs to shared retained geometry,
  not Overtake tuning.

## Classification

- Root cause: current course-frame provenance is constructed only forward from
  measured progress although retained continuity explicitly permits a small
  negative progress delta.
- Contributor: async result age and plant/model progress mismatch expose the
  lower-bound gap.
- Mask: emergency fallback preserves a valid output but makes the defect look
  like unexplained braking.
- Detection gap: telemetry reported the proof reason but not the requested and
  available progress interval.

## Repair evidence

`output/20260824-045351` exercised Follow production and one Overtake episode.
There were zero `course-frame-unavailable` lines in Domain 1. Across eleven
Overtake canonical telemetry windows (396 eligible cycles), 298 reached a
current-world-certified candidate and none failed course-frame reconstruction.

The repair did not turn all retained plans into valid plans. It exposed the
remaining independent proof outcomes:

- stage corridor violation: 39 cycles;
- initial corridor violation: 30 cycles;
- progress discontinuity: 20 cycles;
- corridor horizon unavailable: 6 cycles;
- intent generation mismatch: 2 cycles;
- initial pending: 1 cycle.

These outcomes cluster around the production Mission's side/intent transition.
They remain fail-closed and must be audited as a separate Slice. In particular,
the 39 stage-corridor failures must not be treated as evidence for relaxing the
corridor.
