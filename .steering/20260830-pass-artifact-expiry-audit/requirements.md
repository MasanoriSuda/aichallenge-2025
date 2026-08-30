# Requirements: Pass artifact-expiry audit

## Objective

Classify why an active same-side Pass loses normal authority when its published
artifact expires, despite an opposite sibling branch being certified in the
same worker result. Determine whether the defect is lifecycle/scheduling,
candidate population, single-SQP approximation or physical infeasibility.

## Frozen evidence

`output/20260830-162637/d1/autoware.log`, decisions 5683--5698:

- active Pass side is positive;
- the positive solve reaches maximum iterations;
- the negative sibling is certified from the same worker result;
- cross-side/no-return state prohibits treating that sibling as the executed
  continuation;
- the last positive published artifact then exhausts;
- normal authority is unavailable and Emergency Stop is published.

## Invariants

- Do not change production authority during the audit.
- Do not add Mission resume rules, leases, grace periods, timeouts or fallbacks.
- Do not change solver tolerances, wall clearance or speed parameters.
- An opposite sibling is evidence, not same-side command authority after the
  no-return boundary.
- Compare frozen snapshots through the existing observation-only architecture
  harness before selecting an implementation Slice.

## Acceptance

- Reproduce at least one same-family Pass failure offline.
- Run the existing A--D/bounded-population comparison on the immutable input.
- Classify the first causal layer that produces a certified ManeuverBundle.
- Record a single next implementation boundary, or record that more evidence
  is needed without changing production.
