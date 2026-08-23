# Design

Run ordinary `make dev2` and poll only the one-second Follow telemetry windows. Stop after 200 valid
attempts or after a bounded observation period. Compare the stable live cohort with both the original
82.73% baseline and the isolated 90/90 replay cohort.

The simulator result is redirected to an ignored dedicated directory so the user-owned result JSON is
not modified.
