# Design

## Root-cause hypothesis

The normal worker rebuilds a candidate from the current world, but production
adopts that result as a suffix of an already executing certified plan. The
candidate can therefore solve and receive physical certification yet fail its
later progress/steering join. Public MPCC implementations instead repeatedly
solve from the latest state and apply the first control of that solve.

## Observation-only clock

Add `DirectRecedingCandidateObservation` to the execution-clock vocabulary.
It resolves to cursor zero, but unlike `BootstrapCandidate` it explicitly
means that an executed predecessor exists and that the result is evidence only.
The production adapter rejects this clock unconditionally.

The normal consumer evaluates B only once for each new candidate sequence and
caches that observation. The regular A evaluation and selected production
authority are unchanged.

## Telemetry

Aggregate, rather than print every control cycle:

- distinct candidate observations;
- A accepted / B accepted;
- A-fail/B-pass and A-pass/B-fail classifications;
- average and maximum result age;
- average and maximum control-origin offset from candidate prediction origin;
- last A/B reasons and B progress, steering and pose residuals.

## Safety argument

Even a successful B proof cannot reach command construction through the
production adapter. The only published authority remains the existing A or
executed-plan path.
