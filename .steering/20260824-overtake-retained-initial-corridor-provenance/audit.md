# Root-cause audit

## Observation

In `output/20260824-134024`, generation 1 entered ShiftOut at log line 605
and initially published certified retained canonical control at line 614. At
line 640 normal authority first became unavailable due to
`initial-corridor-violation`; subsequent cycles reported current-origin
discontinuity. Emergency braking therefore began before the later
`ShiftOut -> Pass` transition at line 660.

## Competing hypotheses

| Hypothesis | Supporting evidence | Refutation evidence needed | Confidence |
|---|---|---|---|
| Measured plant lateral pose left the sealed time-zero corridor | Retained proof reports initial corridor violation after lateral motion | measured `ey` remains inside logged bounds | medium |
| Expected retained state is outside the rebuilt corridor | async plan/corridor lifetimes are separate objects | expected `ey` remains inside logged bounds | medium |
| Cursor/progress alignment selected the wrong time-zero slice | progress discontinuity follows shortly after the initial rejection | measured and expected progress align with cursor | medium |
| Corridor is already invalid at time zero | current log has no bounds provenance | finite ordered bounds containing at least one pose | low |

## Missing evidence

`OvertakeCurrentWorldProofResult` currently retains only a categorical reason
and minimum accepted reserve. On rejection it discards both rejected operands
and the sampled bounds. A parameter or transition change would therefore be a
guess.

## Invariant

Every canonical current-world rejection must retain the operands that made the
proof false. Observation must not change proof admission or command ownership.

## Dynamic evidence

Run: `output/20260824-135615/d1/autoware.log`

The diagnostic separates the hypotheses at the first sustained authority
break in episode 2 / Mission generation 1 / target `d2` / side `+1`:

```text
reason=initial-corridor-violation
stage=2
progress=117.946/117.946/118.130
ey=-0.697433/-0.614498
bounds=[-0.681721,0.0182792]
reserve=-0.0157125/0.0672223
```

The three progress values are measured, lifted measured, and retained expected
progress.  The lift is exact and the expected progress is only 0.184 m ahead,
so neither a lap-branch jump nor an invalid progress origin explains the first
rejection.  The bounds are finite and ordered.  The expected retained state is
inside the sealed corridor with 6.7 cm reserve, while the measured plant state
lags laterally by 8.3 cm and is 1.6 cm outside the lower bound.

Immediately before this rejection:

- ShiftOut entered with a certified canonical command;
- fresh and retained canonical selections both succeeded repeatedly;
- the asynchronous receding source later reported solver maximum iterations;
- cycles without an accepted incoming plan had only the stored plan available;
- the stored plan then failed the time-zero corridor proof and explicit
  Emergency braking became the only publisher authority;
- an accepted fresh plan briefly restored `+1.37 m/s2`, after which the same
  stored-plan rejection restored `-3.0 m/s2`.

The later `progress-lift-rejected` and cursor expiry are downstream effects
after emergency braking has already broken execution continuity.

## Hypothesis result

- Measured pose left the sealed corridor: **confirmed**.
- Expected retained pose was outside the corridor: **falsified**.
- Wrong circular branch/cursor origin caused the first break: **falsified**.
- Malformed time-zero bounds caused the first break: **falsified**.

## Root-cause classification

The architectural root is a missing execution-tracking tube contract between
the solved trajectory and its retained current-world proof.  A valid solution
may place its expected state only centimetres inside the dynamic corridor, but
the retained-plan contract has no certified allowance for bounded plant/model,
actuator, and asynchronous handoff error.  When a fresh solve is unavailable
for one control cycle, the stored plan is therefore not reusable even though
its expected state remains valid.

The explicit Emergency supervisor is not the root cause; it correctly refuses
uncertified normal authority.  Its alternating use with intermittent fresh
plans is an important contributor because full braking and re-acceleration
increase the tracking mismatch.  Relaxing `lateral_tolerance_m`, wall margin,
or plan age would hide the missing reachability contract and is not justified
by this evidence.

## Next structural Slice

Make the time-zero/early-stage Overtake corridor a certified reachable tube:

1. derive a bounded lateral tracking envelope from the current measured state,
   current actuation state, and the canonical horizon;
2. require fresh-plan admission to contain that envelope through the handoff
   prefix, rather than certifying only the nominal expected state;
3. store the same envelope in the immutable plan certificate and use it for
   retained revalidation;
4. keep wall and target footprint constraints hard and do not add a grace,
   retry, timeout, or fallback owner.
