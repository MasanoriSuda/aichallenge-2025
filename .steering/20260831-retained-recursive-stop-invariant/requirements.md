# Requirements: retained recursive Stop invariant

## Objective

Prevent retained normal authority from advancing into a state where its direct
current-world Emergency Stop successor is already wall-infeasible.

## Root-cause gate

- Expected invariant: every retained command publication has an exact
  current-world Stop contingency after its unavoidable publisher interval.
- Actual invariant: the Stop contingency is rebuilt only after continuation
  proof has degraded to `PublisherIntervalPrefix`; a `FullSuffix` result can be
  published without revalidating current-state stoppability.
- First visible violation: run `output/20260831-015209`, sequence 2194 was still
  accepted at decision 2852 with five full-suffix stages.  At decision 2856 the
  first terminal check failed, a direct Stop was already wall-blocked, and the
  kart contacted the wall at decision 2876.
- Frozen architecture comparison: changing Return Mission/candidate geometry
  does not help; solved A/B/C/G candidates all fail the same exact Stop proof.

## Constraints

- Do not change Mission resume, lease, grace, timeout or fallback behavior.
- Do not change solver tolerance, clearance, wall margin, costs or weights.
- Do not add a second publisher or normal authority source.
- Reuse the existing exact Stop builder and wall/dynamic/Follow certificate.
- Full-suffix and publisher-prefix authority must obey the same recursive Stop
  invariant.

## Definition of done

- A deterministic full-suffix fixture whose normal path is clear but Stop is
  wall-blocked is rejected before authority.
- A clear full-suffix fixture includes a certified Stop in its proof.
- Existing partial-prefix behavior remains unchanged.
- Production adapter accepts the resulting proof without a new branch.
- Package build and full tests pass.
- A dynamic run shows terminal rejection before wall contact; race-quality
  acceptance remains a separate Gate.
