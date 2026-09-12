# r72 source admission observation

2026-09-12 JST. Baseline and rollback `ac1622d8`. Authorized autonomous M1–M6;
local commits, no push. Overall acceptance remains open.

Standard dev2-r72 moves both cars then remains stopped. D1 reaches1.0729107857m/s
and61 consecutive positive publications/1.5s; D2 reaches1.4106055498m/s and66/1.625s.
These producer observations do not establish actual application or restart.
No laps complete. The manually stopped run is rejected; original DLL and user
JSON restoration is recorded. D1 has one32.002811ms callback overrun; D2 has none.
The runner's moving-publication detector does not cover stationary failures.

D1decision2229 at48.034998926 has zero plans/context rejections, but the Store
has accepted99 candidates and advanced to1727. Atdecision8657 the Store reaches
8130/354 accepted with the same zero-plan boundary. D2 instead retains Store645
while current geometry changes to821299624407504693; its captured731 solver input
belongs to15879444219207335847 and must not substitute for the missing new world.

The earliest unresolved D1 boundary is `submit_scheduled_post_publication`:
the combined plan/artifact/source/generation guard returns without a reason.
Normal population candidates copy the source generation and certified pipeline
stores a source snapshot. These code facts do not identify the live invalidator.
`last_solver=unobserved` is an unconsumed telemetry-window field, not the worker's
latest result. No generation bypass, retry, source reuse or solver change is
justified by this observation gap.

The observation slice splits the existing guard without changing its order or
conditions. Per-decision missing plan/artifact/source and rejected/invalid
generation counts are emitted by the existing post-publication cadence. The
control-thread invalidator records its caller, decision and cumulative calls;
these fields never participate in authority. Mailbox state reports the latest
published result's outcome, geometry and at most512 detail characters under its
existing lock, without copying a trajectory. The old telemetry fields retain
their meaning. Empty/stale mailbox checks must preserve sequence ownership.

Separately, D1Follow decision1168 preserves source673/job1165, current geometry
15741568466272881844 and exact publication history. Callback32.002811ms contains
current proof30.992227ms wall/30.074677ms CPU. The packet's nominal17.954999609,
entry17.959999598, actual before17.989999597 and original deadline17.979999609
establish a correct expiry rejection. Built-library replay accepts physics and
entry but rejects the actual before clock, with951 current samples and about20ms
isolated current work. Composite Follow has no independent domain under the
existing proof policy. Its numerical cost is a separate unresolved contributor.

Validation: existing source contracts, mailbox sequence/result ownership checks,
full package tests, `make autoware-build`, and committed standard dev2-r73 for
the first missing boundary. No control rates, receiver/publication windows,
130ms origin,100ms steering, model, margins, solver or normal authority change.
Follow cost profiling may use diagnostic copies with unchanged captured inputs.
Observation alone is not a repaired production candidate. After attribution,
repair the demonstrated producer and retain or remove only necessary diagnostics.

Implemented and locally validated: build114all26packages, tests101all2592records/
66groups/source105/C5 161, no errors/failures/skips. The bounded mailbox observation
preserves the latest sequence across a rejected stale publication. Standard r73
and producer attribution remain next. [Evidence](r72-source-generation-evidence.json).

R446 diagnostic setup failed on a multiline function signature; R447 corrects
only that locator and retains the original source/history/world/clock outcomes.
Profile medians in milliseconds: {"advance_partitioned_inputs": 12.168344999999983, "centered_step": 11.686393000000058, "complete_current_reproof": 21.519923, "follow_progress": 3.821794999999989, "force_coefficients": 0.997124999999995, "step_parts": 11.908083999999965, "world_sample": 8.763420999999967}. These are nested scopes,
so they must not be added. Native propagation and current-world checks both
contribute; Follow projection is only part of the world cost. No numerical
optimization is promoted in this observation slice.
