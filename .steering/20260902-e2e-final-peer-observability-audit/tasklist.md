# Tasklist

- [x] Freeze diagnostic-only corpus and representation audit.
- [x] Make the audit speed freshness contract explicit and recorded.
- [x] Run the final-peer observability report.
- [x] Compare against the previous teacher/normal conflict audit.
- [x] Record root-cause classification and next architecture boundary.

## Decision

The final four-domain peer labels are substantially more distinguishable from
normal driving than the historical seed2033/2034 labels.  The unresolved
boundary is evaluation coverage: all peer domains come from one training run,
while both fixed validation worlds use the older, more ambiguous distribution.
Do not tune another model until an independent successful peer world is
collected as validation-only evidence.
