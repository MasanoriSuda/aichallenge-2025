# Design: ShiftOut terminal-Stop authority loss

Use the closest recorded pre-loss interaction:

`output/20260831-153609/d1/mpcc_architecture_snapshots/`
`000000000890-d416b709508d0dce-shiftout-side-negative-`
`dynamic-obstacle-refinement-solve-rejected/snapshot.yaml`

The snapshot was captured at control decision 1525, approximately 0.15 s
before the first terminal-Stop publication at decision 1531.  It represents
the alternate negative homotopy while the committed Mission side was positive;
the comparison must therefore report both stateless homotopies rather than
assuming the snapshot side is the only candidate.

Correlate the replay with:

1. the last forward normal publication;
2. the first certified terminal-Stop selection;
3. the first `steering-unreachable` normal rejection;
4. the later target-loss and Recovery transitions.

## Frozen comparison result

The unchanged full comparison completed against source sequence 890.

| Arm | Result | Relevant evidence |
| --- | --- | --- |
| A: persistent Mission | rejected | dynamic-obstacle lateral row, stage 5, maximum iterations |
| A2: persistent geometry/current target | rejected | exact wall proof rejected |
| B: stateless left/current world | accepted | candidate `5964043013132420366`, wall/dynamic/terminal Stop certified |
| B: stateless right/current world | rejected | exact wall proof rejected |
| G: production left/current world | accepted | same candidate and proof result as B |
| G: production right/current world | rejected | exact wall proof rejected |

The C/D families were also executed.  Their outcome cannot change the first
discriminating result: A failed while B succeeded on the same immutable world,
same seven-state formulation and same exact proof gates.  The frozen state was
therefore physically executable without changing a solver tolerance or a wall
clearance.

## Runtime chronology

- At decision 1524 the publisher was still advancing artifact 771, originally
  solved at decision 1406, at stage 18 of 20.
- At the same time the current frozen Mission path was 5.8 seconds old and its
  DP execution authority had already been released.
- The latest active-Overtake producer evidence showed the selected branch
  rejected while its sibling was certified, but the producer did not publish
  either result until both branch computations joined.
- At decision 1531 artifact 888 contributed only its certified terminal Stop.
- At decision 1532 the normal join was `steering-unreachable`; Emergency Stop
  then became the effective authority.
- Target stale/lost and Recovery occurred later and are downstream symptoms.

## Root cause

The active-Overtake dual evaluator coupled the committed primary branch to the
completion of the opposite observation branch.  It evaluated one side on a
bounded executor, evaluated the other side in the latest-only producer, and
then called `wait()` before admitting the selected plan to the certified Store.
Consequently a slow or difficult alternate branch delayed a currently feasible
same-side stateless bundle.  The previously published artifact reached the end
of its usable stages first and the correctly certified Stop suffix took over.

The old persistent Mission geometry amplified the failure, but the first
production invariant violation was scheduling/lifecycle: an independent
alternate candidate failure was allowed to delay normal primary authority.
This is the `offline succeeds but live fails` exit classification.

## Structural repair

Use the same ownership rule already used by normal Cruise/Follow avoidance:

1. evaluate the committed Overtake side as the primary producer;
2. admit its complete certified plan immediately;
3. evaluate the opposite homotopy on the bounded sibling executor;
4. merge sibling evidence into the same-epoch branch bank without waiting;
5. never let sibling failure clear or delay the primary Store admission.

This repair changes no Mission side, no no-return decision, no physical proof,
no solver setting and no clearance.  The sibling remains observation-only.
