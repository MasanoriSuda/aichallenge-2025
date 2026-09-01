# Design

## Root cause

AWSIM produces race JSON in its process current working directory.  The
simulator wrapper redirected stdout to `LOG_DIR/awsim.log`, but it did not
change cwd from `/aichallenge`.  Consequently repeated development runs
overwrote tracked root JSON, result files were absent from the owning run, and
`output/latest` could retain stale links.  A later motion analysis could then be
combined manually with race results from a different run.

## Correction

Resolve both the simulator script directory and `LOG_DIR` before launch, change
cwd to the immutable run directory, then exec the existing mode script.  Mode
scripts already invoke AWSIM and scenarios through absolute paths, so this does
not change simulator arguments or the ROS/evaluation interface.

Add `analyze_e2e_competition.py` as a read-only aggregation layer:

```text
/output/<run>/result-summary.json
/output/<run>/dN-result-details.json
/output/<run>/dN/e2e-run-analysis.json
/output/<run>/dN/autoware.log
                     |
                     v
       e2e-competition-analysis.json
```

For every requested domain it verifies:

- motion analysis schema and stall admission
- result detail schema, domain identity, Finish, lap count, and penalties
- result summary agreement for Finish and lap count
- a single launch-time checkpoint path and control mode

The source checkpoint SHA is recorded independently.  It proves the selected
artifact identity but does not claim that a host-side file hash alone proves
the bytes loaded inside a historic container; runtime path and launch mode are
therefore retained separately.

## Non-goals

- no model training or promotion
- no longitudinal/lateral policy changes
- no inference feature changes
- no result JSON schema changes
- no inference from stale root-level JSON for old runs
