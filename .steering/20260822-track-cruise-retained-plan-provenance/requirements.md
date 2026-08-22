# Track/Cruise retained-plan provenance

## Baseline

- Branch: `develop_july`
- Baseline commit: `6975cb4`
- Preserve `aichallenge/result-summary.json`.

## Root cause found during authority-contract audit

The canonical selector correctly keeps a retained solution's original problem identity, but an
unexpired original wall certificate does not prove that the vehicle's current pose can still reach
the remaining plan prefix. A delayed callback, localization movement or consumed stages can make the
old connector invalid while the original solution metadata remains certified.

Accepting retained authority from only:

```text
old problem + old certificate + claimed remaining stage count
```

would recreate the late-handoff defect fixed in the previous Slice.

## Required correction

- Give every executable plan a nonzero identity.
- Attach the decision ID of the most recent current-pose execution certificate.
- Require both fresh and retained candidates to have execution provenance for the current decision.
- Preserve the retained solver problem identity separately; do not reseal an old solution as a new
  solve.
- Distinguish missing plan, missing control stages and stale execution certificate in trace reasons.

## Non-scope

- No retained-plan storage implementation.
- No wall revalidation implementation.
- No runtime selector connection or authority promotion.

## Exit gate

- A retained solution with only its original certificate is rejected.
- The same retained problem is accepted after current-decision execution revalidation.
- Fresh candidates also require current-decision execution provenance.
- Build and complete tests pass.
