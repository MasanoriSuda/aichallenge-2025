# Design

This Slice exercises the exact production contracts without granting authority:

```text
stored immutable plan
  -> resolve_execution_cursor(now)
  -> CanonicalExecutionRevalidation(current decision + physical proof)
  -> build_canonical_normal_candidate()
  -> resolve_canonical_normal_authority(fresh, no retained)
  -> require FreshCertified
  -> telemetry only
```

Because this happens in the same decision that created the plan, the swept physical certificate is
current and the cursor must start at stage zero. Later Slice work will revalidate retained plans
against a newer measured pose and observation generation.

The candidate carries metadata and executable-window identity. It does not carry or publish a
legacy command vector.
