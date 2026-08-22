# Design

The Track/Cruise shadow evaluator remains the sole producer in this Slice:

```text
five-state OSQP result
  -> lateral constraint contract
  -> world-frame swept wall certificate
  -> CertifiedMpccSolution bound to the same problem fingerprint
  -> direct CanonicalExecutionPlan extraction
  -> atomic CanonicalExecutionPlanStore replacement
  -> telemetry only (selected=0)
```

The plan ID and solution ID use the control decision ID. There is exactly one Track/Cruise shadow
evaluation per decision, and the decision sequence is monotonic for the controller lifetime. The
store independently rejects stale IDs and retains its high-water mark across snapshot clears.

The optional legacy conversion remains only for model-difference telemetry. Canonical storage does
not depend on that lossy conversion succeeding.

Authority selection, current-window revalidation and command extraction remain outside this Slice.
