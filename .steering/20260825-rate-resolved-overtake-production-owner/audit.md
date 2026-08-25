# Root-cause audit

## Observed phenomenon

The first ShiftOut in `output/20260825-143421` never reached Pass. Normal
authority alternated with long runs of Emergency, and the vehicle entered
external Recovery.

## Causal chain

At decision 5593, the retained five-state plan requested steering `0.366512`
rad while the command predecessor was `0.324346` rad and the 25 ms reachable
interval was `[0.306846, 0.341846]`. The plan was rejected rather than mutated.

Subsequent Emergency commands held steering while commanding zero speed. The
measured lateral state moved from roughly `-0.61 m` to `-1.07 m`; the frozen
Mission goal remained `+0.84 m`. Current-world proof then correctly rejected
the immutable suffix with `initial-corridor-violation`, negative reserve, and
later progress discontinuity.

The visible stop and Recovery are downstream symptoms. The earliest defect is
the mismatch between coarse five-state curvature stages and the 40 Hz steering
actuation time base.

## Existing-patch relationship

The continuity and current-world checks are not the defect. They correctly
prevent an unreachable or already departed plan from owning control. Relaxing
them, clamping steering, or retaining the plan by age would hide the authority
mismatch and publish an uncertified trajectory.

The repository already contains the structural replacement: the six-state
rate-resolved pipeline used by Track/Cruise. Overtake still bypasses it through
its older five-state production function.

## Chosen correction

Migrate Overtake normal authority to the same six-state producer and causal
post-commit snapshot ordering. Do not change tactical parameters or add another
fallback.

## Integration defects found during the migration

Three old ownership assumptions initially blocked the replacement:

1. DynamicEscape was logged as ShiftOut but the canonical problem identity was
   populated only from OvertakeLine, producing Idle/side-zero problems.
2. production adaptation and retained current-world proof each carried a local
   Track/Cruise-only intent predicate even after the execution artifact had
   gained Overtake support.
3. first-cycle admission depended on the failure reason of an artifact from
   the previous intent. After Follow, an exhausted Cruise artifact therefore
   prevented a new ShiftOut solve.

These are authority and identity defects, not parameter defects. The correction
uses one canonical identity resolver, one artifact intent predicate, and an
intent-transition admission which still passes the same solver, exact adapter,
physical-wall proof, and current-world join.

## Dynamic evidence after correction

In `output/20260825-160839/d1/autoware.log`, ShiftOut admissions include:

```text
decision=4739, previous=follow, intent=shiftout, certified=1,
solver=solved, physical=accepted
solver="canonical-rate-resolved-shiftout-retained"
formulation=velocity-steering-progress-6state
```

The run contains 21 physically certified ShiftOut admissions and 17 retained
six-state ShiftOut publications. Other candidates were rejected with explicit
`stage-wall-rejected` or semantic-artifact reasons and went to Emergency; the
old five-state normal producer was not reintroduced. Pass and Return did not
occur before shutdown and therefore remain dynamic-evidence debt.

## Remaining concern

The run still shows many physically rejected candidate admissions and brief
target-loss periods while the tactical action remains DynamicEscape. Those are
observable upstream tactical/path-quality issues. They are deliberately not
masked in this authority Slice with a grace period, clamp, five-state fallback,
or tuning change.
