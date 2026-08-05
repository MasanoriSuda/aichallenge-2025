# Design

## Admission

The second-stage handoff preflight retains its frozen goal.  Its nominal ramp
uses `max(current ego speed, candidate overtake command speed)` so a mission
entered at follow speed is not planned as if it will remain at that speed.

## Runtime

The scheduled transition passes the remaining transition-window distance to
the continuation planner.  The continuation planner then calculates:

```text
available shift = min(configured maximum,
                      remaining scheduled window,
                      remaining absolute Pass budget)
required shift  = f(current lateral error,
                    current ego speed,
                    lateral acceleration limit)
```

The existing atomic continuation checks validate the newly sized ramp and the
remaining Pass/Return path before changing the committed side.

The admission ramp remains in mission state only as a diagnostic nominal
value.  It no longer truncates the runtime ramp.

## Failure diagnostics

An acceleration-budget rejection reports current speed, lateral adjustment,
required distance and available distance.  Scheduled transition logs report
both the nominal admission ramp and the live available window.

