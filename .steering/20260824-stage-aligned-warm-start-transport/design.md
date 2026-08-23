# Design: stage-aligned warm-start transport

## Causal chain

```text
previous certified solution
  -> identity resolver proves exact or rolling spatial overlap
  -> resolver discards the overlap offset
  -> five-state solver always shifts one stage
  -> primal/dual rows no longer describe the current spatial grid
  -> OSQP reports solved under aggregate residuals
  -> physical row certificate rejects acceleration/velocity
  -> Emergency authority for that cycle
```

The rejection is observed downstream, but the earliest deterministic defect is
the lost stage correspondence at the warm-start boundary.

## Change

1. Extend `ShadowWarmStartResolution` with the proved stage offset.
2. Replace the boolean rolling-compatibility helper with an overlap resolver
   which returns that offset.
3. Add a generic `transport_mpc_warm_start_by_stages()` transformation.
4. Keep the existing `shift_mpc_warm_start()` as the explicit one-stage legacy
   wrapper.
5. Pass the canonical offset into Follow and Track/Cruise five-state solves.
6. Expose the selected offset in decision telemetry.

The progress-coordinate rebase remains after spatial transport because the
five-state progress variable is relative to the current measured progress
origin.  Certified-only publication remains unchanged.

## Rejected alternatives

- Relax execution-primal tolerances: hides the row mismatch.
- Add a cold retry after warm rejection: restores a second execution path and
  violates the single-authority migration rules.
- Always disable warm starts: removes the symptom but abandons the persistent
  solver design and does not correct the identity contract.
- Shift by elapsed wall-clock time alone: the QP horizon is built from spatial
  waypoint stages; the existing exact spatial identity is the authoritative
  alignment source.

