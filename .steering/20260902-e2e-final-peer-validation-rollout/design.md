# Design

Launch `e2e-final-speed-committed-teacher-all` with a fixed explicit output
directory.  Wait until all four domains are Grounded, publish the existing
AWSIM start request, and monitor Finish/result artifacts.  If vehicle-side
postprocessing again fails to observe FinishALL, shut down cleanly only after
all result JSON files exist, then analyze the finalized bags.

Reuse the existing strict competition and motion analyzers.  The run remains
outside training regardless of outcome.  A passing replicate can become a
new validation source in a later dataset Slice.
