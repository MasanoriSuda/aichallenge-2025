# Requirements

## Purpose

Promote the certified rate-resolved six-state formulation to the sole normal
Track/Cruise publication owner and remove the five-state Track/Cruise owner in
the same Slice.

## Root cause

Track/Cruise currently solves and publishes the five-state canonical plan even
when a retained, current-world-certified six-state command is available. The
six-state command stops at a shadow-only candidate type, while the final
publisher and execution trace accept only the five-state command path. Keeping
both owners behind a selection flag would preserve the architecture defect.

## Required invariants

- A Track/Cruise normal command is published only from a current-world-certified
  six-state retained proof.
- The exact source problem, solution identity, execution plan and current
  execution-certificate decision reach the final control trace.
- Track/Cruise no longer invokes the five-state canonical solve or uses it as a
  normal fallback.
- Missing or rejected six-state authority produces explicit EmergencyStop; it
  never silently selects another formulation.
- The next asynchronous problem is bound to the steering actually selected for
  publication in the current cycle.
- A certified six-state actuation is already inside the original physical
  input envelope exactly; the publisher must not repair or clamp it.
- Follow, Overtake, Emergency and Recovery authority are unchanged.
- No parameter, tolerance, timeout, fallback flag or ROS interface changes.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Failure-first tests cover six-state canonical command identity and reject
  five-state-owner re-entry in Track/Cruise.
- Source search proves the Track/Cruise branch cannot call the five-state
  canonical evaluator.
- Package tests and `make autoware-build` pass.
- A short `make dev2` run shows six-state Track/Cruise production publication,
  complete final trace identity and zero postprocessor mutation. Any callback
  overrun or missing dynamic observation is recorded as a separate, typed
  follow-on risk instead of being hidden by a cross-formulation fallback.
