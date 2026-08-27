# Mission lifecycle A/B comparison requirements

## Baseline

- Branch: `develop_july`
- Frozen source baseline: `0287d1930bfdd7b89c36a25cb9f75bd76800d762`
- Production authority must not change during this comparison.

## Objective

Determine whether an observed Overtake failure is caused by retaining and
executing a persistent Mission lifecycle, rather than by the common seven-state
SQP formulation or by physical infeasibility.

## Compared pipelines

- A: the active persistent Mission candidate and the canonical seven-state SQP.
- B: a fresh receding candidate rebuilt from the same tactical world snapshot,
  treated as a stateless ManeuverBundle, and the same canonical seven-state SQP.

## Controlled variables

An A/B sample is comparable only when both pipelines share:

- one immutable world/decision identity;
- target identity and target-observation generation;
- homotopy/side identity;
- source phase and source Mission generation;
- vehicle state, reference path and wall snapshot;
- seven-state model, horizon, weights, solver settings and hard constraints.

The candidate trajectory is intentionally different.  A uses the retained
Mission trajectory while B uses the current-world receding trajectory.

## Prohibited changes

- No Mission resume rule, lease, grace period, timeout or fallback.
- No solver tolerance, clearance, wall-margin or performance tuning.
- No production authority, branch selection, published command or state
  transition may depend on the A/B result.
- No classification from two different run times or unmatched world snapshots.

## Exit classification

- A fails and B succeeds: Mission lifecycle defect.
- A and B fail: not sufficient to distinguish candidate-generation,
  single-SQP or physical infeasibility; continue with C/D.
- A succeeds and B fails: retained Mission is not the cause in that snapshot;
  investigate B candidate generation.
- Both succeed: no reproduced failure; sample is non-diagnostic.
- Solve succeeds but proof fails: model/certificate mismatch.

## Definition of done

- The A/B candidate contract is covered by unit tests.
- Dynamic comparison records contain a common immutable world key and separate
  A/B problem outcomes.
- At least one Pass or Return failure-class snapshot is compared, or the lack
  of a comparable fresh B candidate is reported explicitly.
- A production-authority diff audit confirms that no A/B result is consumed.

