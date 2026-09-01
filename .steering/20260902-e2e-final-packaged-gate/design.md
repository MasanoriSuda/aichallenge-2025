# Design

Run `make e2e-final` with an immutable output directory, wait until all four
Autoware domains have reached the AWSIM synchronization barrier, then publish
the existing `/admin/awsim/start` request.  Let the 420-second session terminate
naturally so AWSIM writes authoritative result JSON.

For each domain:

1. generate `e2e-run-analysis.json` from its bag;
2. integrate Finish, lap, penalty, motion and launch provenance with
   `analyze_e2e_competition.py`;
3. compare clean-lap pace with the 70 s/lap minimum required to complete six
   laps within 420 seconds;
4. freeze the earliest causal failure before proposing another Slice.
