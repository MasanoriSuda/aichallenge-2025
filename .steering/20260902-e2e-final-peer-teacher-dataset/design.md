# Design

The four-domain teacher race proves that the policy can resolve the actual
peer-interaction world.  It does not prove that the current recurrent student
can represent those decisions.  Reuse the existing certified relabeler so the
same policy is replayed sequentially with causal speed and exact outcome
provenance.

All four domains belong to the training side of one world-level group.  A
domain-level validation split would leak the same interactions and course
timeline into both sides.  Existing seed-2033 teacher and production-normal
validation sequences remain independent gates.

The historical offline contract used a stricter 50 ms speed age, but the
executing controller's recorded runtime contract is 100 ms.  Three d1 scans
exceeded 50 ms, so the first extraction was rejected rather than silently
dropping temporal samples.  This dataset explicitly records a 100 ms maximum,
matching the runtime that actually produced the successful commands.  The
global 50 ms defaults are not changed.

Before training, compare:

- material correction and neutral-anchor counts;
- decision and temporal-supervisor reason distributions;
- causal speed age;
- label magnitude and sign balance;
- coverage relative to the one-ego/two-NPC source.

Only a material peer-interaction coverage gain justifies a new recurrent
candidate Slice.  Extraction itself does not change runtime authority.
