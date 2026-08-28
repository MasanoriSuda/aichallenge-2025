# Design

`record_failure()` already seals a complete v2 interaction snapshot, but its
directory name used only `identity.sequence`, intent, stage and outcome.
Candidate sequences restart, so a later process could hit the same directory.
The recorder then discarded its temporary current-world artifact and returned
the old pathname as if the new snapshot had been written.

Add the interaction fingerprint to replay-ready directory names. The existing
per-process family key still limits write volume; this change only prevents
cross-run aliasing. Incomplete exact-QP-only evidence retains the legacy name
because it has no replay-ready interaction fingerprint.
