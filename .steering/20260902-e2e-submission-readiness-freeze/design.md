# Design

`audit_e2e_submission_readiness.py` consumes existing immutable evidence
instead of reinterpreting logs ad hoc:

1. hash the packaged raw and spatial artifacts;
2. require the single-vehicle competition report to pass;
3. read the mixed-peer motion admission independently;
4. read the privileged future-occupancy oracle independently;
5. emit a three-state decision.

The states are:

- `reject`: artifact identity or single-vehicle qualification failed;
- `single-vehicle-candidate-only`: single passes but mixed-peer evidence fails;
- `multi-vehicle-candidate`: both single and peer Gates pass.

Oracle evidence is diagnostic only.  Even a discriminating oracle does not
change production readiness; label generation additionally requires an
explicit admissible supervision contract.  The current oracle forbids labels
and runtime input.
