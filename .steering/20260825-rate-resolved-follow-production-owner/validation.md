# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `d4213bd refactor(mpcc): remove five-state overtake entry authority`
- First dynamic run: `output/20260825-171642`
- Corrected dynamic run: `output/20260825-172643`
- User-owned `aichallenge/result-summary.json` was preserved and is excluded
  from this Slice.

## Failure exposed by the first dynamic run

Static build and tests initially passed after routing Follow to the shared
normal owner, but the first `dev2` run exposed an incomplete vertical join:

```text
V2X behavior: Cruise -> Follow
Rate-resolved normal submission unavailable:
  intent=follow, reason=rate-resolved request unavailable
canonical-follow-emergency
```

The submission boundary recognized Follow as six-state scope, while
`build_extended_progress_problem()` only allocated the semantic rate-resolved
request for Track/Cruise or Overtake.  Follow therefore had a valid
longitudinal contract but no request object.  Emergency was correctly selected
instead of a five-state fallback, but Follow production authority was not yet
complete.

## Root cause and structural fix

The same normal-scope policy was independently reimplemented at two layers and
the older lower-layer copy omitted Follow.  A typed
`request_scope_available()` resolver now owns the mapping:

- Track/Cruise -> racing semantics;
- Follow -> Follow semantics;
- ShiftOut/Pass/Return -> Overtake execution semantics;
- unsupported intent -> unavailable.

Both semantic request assembly and submission admission call this resolver.
Failure-first coverage demonstrated the API was absent before the fix and
checks all owned and unsupported intents after it.

The old five-state Follow retained path had also owned a target-specific hard
gap proof which the generic six-state physical obstacle proof did not replace.
The shared retained evaluator now requires a current typed Follow target,
checks target identity/generation against the current dynamic-world snapshot,
and verifies the current plus all remaining stage gaps using effective ego
progress (`progress + lag`).  No hard-gap parameter was duplicated or tuned.

## Removed authority

The dedicated five-state Follow lifecycle, worker, solver context, plan store,
transition admission, telemetry, and publisher path were physically removed.
Static search reports zero remaining old Follow-owner symbols.  Tactical
Follow contract generation remains as semantic input to the shared six-state
solver; it is not command authority.

## Static validation

- Failure-first compile: failed on the absent shared scope resolver as
  expected.
- Focused retained/source-contract tests: 2/2 passed after the fix.
- `make autoware-build`: passed, 25 packages.
- Package CTest: 49/49 targets passed.
- `git diff --check`: passed.
- No speed, gap, wall, solver, weight, timeout, horizon, cadence, Recovery, or
  ROS-interface parameter was changed.

## Dynamic validation

In corrected run `output/20260825-172643/d1/autoware.log`:

- Follow six-state certified publication appeared 8 times in the decision
  trace;
- retained Follow proof telemetry appeared 12 times with 21 checked states,
  current observation generation, and a finite minimum Follow gap;
- `Rate-resolved normal submission unavailable: intent=follow` appeared zero
  times;
- representative final trace reported
  `intent=follow`, `formulation=velocity-steering-progress-6state`,
  `authority=certified-normal-solution`, `retained=1`, and
  `canonical=satisfied`;
- one isolated Follow cycle failed closed to explicit Emergency when retained
  evidence was temporarily unavailable, then returned to the same six-state
  owner.  No different normal formulation was selected;
- callback-overrun detail count was zero during the bounded evidence window.

The shutdown sequence later terminated RViz and the autostart orchestrator;
there was no `mpc_controller_cpp` process death in the acceptance window.

## Result

Follow now shares the same normal six-state formulation, worker/store,
current-world retained proof, production adapter, and publisher as
Track/Cruise/ShiftOut/Pass/Return.  The Slice exit gate is satisfied.  Rejoin
and residual five-state representations remain a separate audited Slice; they
must not be mixed with parameter tuning.
