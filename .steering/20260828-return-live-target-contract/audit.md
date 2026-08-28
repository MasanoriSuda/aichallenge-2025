# Audit

## Observation timeline

| Time | Observation |
|---|---|
| 84.641 s | `Pass -> Return`, rear-clear confirmed, live Return preflight committed |
| 84.749 s | certified Return artifact 4117 published at 6.17 m/s |
| 84.769 s | fresh Return submission first reports incomplete source context |
| 85.099 s | Return artifact 4119 is solved, physically accepted and published |
| 85.216 s | behavior has no active target; completed d2 is behind ego |
| 89.142 s | 81/81 new submissions rejected; artifact 4119 cursor exhausted |
| 90.444 s | canonical Return Emergency Stop is the final control owner |
| 90.944 s | stuck detector confirms the induced stop |
| 91.269 s | Return ends only through external Recovery |

## Hypotheses

| Hypothesis | Support | Falsifier | Confidence |
|---|---|---|---|
| Physical Return is infeasible | later pose could violate wall/obstacle proof | artifacts 4117/4119 solved and passed physical certification | rejected |
| Single-SQP cannot solve Return | later curvature may be difficult | fresh Return solves succeeded until identity construction stopped | rejected |
| Async scheduling lost the result | worker result could be stale | no new jobs were admitted; 81 submissions were rejected before solve | low |
| Return identity incorrectly requires live target provenance | rejection starts when completed target leaves active set; context contract requires target generation | a complete Return context with target generation zero would still fail upstream | high |

## Classification

This is a Mission lifecycle / semantic identity defect. A physically feasible
Return exists, but the producer stops rebuilding it because the completed
target's tactical observation is conflated with the encounter identity.

## Next Gate

The repaired run must show continuing `intent=return` submissions after the
V2X behavior target becomes empty, then a normal `Return -> Idle` handoff. Any
new wall, convergence or solver failure after that point becomes separate
evidence; it is not preemptively patched in this Slice.

## Static validation

- `make autoware-build`: 25 packages passed.
- Correct overlay CTest: 52/52 passed.
- The single-authority structural audit now checks the explicit encounter-ID
  responsibility rather than relying on the old local variable name.
- No parameter, solver, fallback, timeout or lease was changed.

## Dynamic validation attempt

Run `output/20260828-135832` exercised six Overtake episodes on domain 1.
None reached `Pass -> Return`, so this run neither accepts nor rejects the
rear-clear Return repair:

| Episode | Last relevant transition | Classification |
|---|---|---|
| 1 | `ShiftOut -> FollowPrepare -> Recovery` | live corridor unavailable, then static wall infeasible |
| 2 | `ShiftOut -> FollowPrepare -> Idle` | Pass-entry wall gate unresolved |
| 3 | `ShiftOut -> Pass -> Recovery` | `SafeSeparation` short horizon unsafe |
| 4 | `ShiftOut -> FollowPrepare -> Idle` | SafetyBrake pause and Mission time limit |
| 5 | `ShiftOut -> FollowPrepare -> Idle` | live corridor unavailable |
| 6 | `ShiftOut -> Recovery` | locked target stale or lost |

Observed counts were `Pass -> Return = 0`, `Return -> Idle = 0`,
`intent=return = 0`, canonical Return emergency = 0 and external Recovery
completion = 0. The sole `source context incomplete` warning belonged to
ShiftOut, not Return.

The dynamic Gate therefore remains open. The upstream Pass-admission and
completion failures are frozen as independent evidence and are not patched in
this Slice. The next qualifying run must reach rear-clear so the repaired
Return producer can be observed after the tactical target disappears.
