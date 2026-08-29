# Results

## Observation

Run `output/20260830-024504/d1/autoware.log` reproduced the frozen failure
without changing production authority or configuration.

- Episode 1 entered ShiftOut at `1788025559.233708`.
- The first Pass-entry gate was held at `1788025560.456066` and expired
  after `0.68 s / 3.03 m` at `1788025561.130687`.
- The fresh same-side Mission entered ShiftOut again; its gate was held at
  `1788025568.480086` and expired after `1.03 s / 3.05 m` at
  `1788025569.505256`.
- Both transitions ended in DynamicMissionWait with
  `Pass entry physical wall gate unresolved`.

The replay material is preserved beneath
`output/20260830-024504/d1/mpcc_architecture_snapshots/`. The two compared
sealed worlds were:

- sequence 1799, fingerprint `6833516978660140820`;
- sequence 2033, fingerprint `8077313769501052827`.

## Same-snapshot comparison

### Snapshot 1799

- A persistent and A2 retained-persistent both failed in the unchanged
  seven-state SQP at an input/steering-rate row.
- B stateless-left failed at the same boundary.
- B stateless-right solved but did not certify because the wall proof had no
  course frame at the progress boundary.
- C rough candidates did not produce a certified Bundle.
- F physical-diagonal-right produced multiple certified Bundles.
- G production-right produced a certified Bundle from the bounded
  `late-exact-disjunction` candidate: terminal progress `14.2794 m`, terminal
  velocity `5.31141 m/s`, lateral reserve `0.266463 m`, solve time
  `61.3765 ms`.

### Snapshot 2033

- A persistent and A2 retained-persistent were rejected at a dynamic
  obstacle lateral constraint at stage 19.
- B stateless-left had the same rejection.
- B stateless-right solved but exact dynamic proof rejected a new predicted
  overlap at `t=3.27215 s`, clearance `-0.00258932 m`.
- C rough candidates did not produce a certified Bundle.
- F physical-diagonal-right again produced multiple certified Bundles with
  terminal progress near `19 m` and lateral reserve about `0.37-0.41 m`.

D was not required. A bounded candidate using the production seven-state
solver and the unchanged exact certificate already proved that the frozen
world was physically executable.

## Live scheduling evidence

During the matching live interval, the Track/Cruise worker reported:

- about 80 submissions per telemetry window;
- 73-76 pending replacements;
- only 5-7 consumed results;
- candidate population compute time about `115-135 ms` on average and
  `194-220 ms` maximum.

The 40 Hz caller therefore replaces more than 90 percent of requests before
the bounded population completes. Most completed results are no longer the
current semantic request, while the exact same sealed world can produce a
certified production candidate offline. The Pass gate then expires as a
downstream symptom.

## Upper-log comparison

`.steering/ano/autoware - 2026-08-21T211659.829.log` records a main GMPCC with
`N=20`, `dt=0.12 s` and a `2.4 s` horizon. Its main solves are commonly
`25-50 ms` and are issued at a sustainable cadence near 7 Hz. Tactical async
work is isolated in a spawned child process. It does not require every
tactical population to finish within the 40 Hz publisher period.

## Classification

Primary classification: **offline succeeds but live fails --
scheduling/lifecycle defect**.

Contributing defect: the candidate/disjunction population contains a
certifiable late-exact solution, but its bounded computation and publication
lifecycle are coupled to a 40 Hz latest-only request stream. This is not:

- a wall-clearance or tuning defect;
- physical infeasibility;
- a seven-state SQP limitation;
- evidence that another Mission timeout or resume rule is needed.

The next production Slice must separate the sustainable candidate producer
cadence from 40 Hz command publication. A completed candidate may only gain
normal authority after immutable problem compatibility, exact current-world
wall/opponent proof and terminal-successor proof. The publisher continues to
use the last actually published certified artifact while a newer compatible
population is being solved.
