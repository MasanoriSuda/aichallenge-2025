# Failure causal trees

These trees distinguish confirmed structural paths from hypotheses requiring replay. A visible
`solver failure`, `wall hold`, deceleration, or Recovery is not automatically the root.

## Tree A: MPC/MPCC formulation churn

```text
Behavior/phase or Dynamic Escape lease changes
  -> progress-contouring activation changes                         [Root: I-04]
  -> problem formulation/warm-start domain changes
  -> extended solve accepted, requalifying, or unavailable
       -> unavailable/requalifying selects 3-state/legacy solve     [Root: I-05]
       -> handoff smoothing / circuit breaker / reentry gate        [Mask]
  -> different first input and prediction ownership across cycles  [Contributor]
  -> downstream speed/steering filter sees a discontinuity
  -> visible hesitation, braking, or lateral authority change      [Symptom]
```

Confirmed by source and by the `legacy-mpc-solved` / `extended-mpcc-solved` transitions in run
`20260822-105057` D1. The exact contribution to a specific collision remains a replay question.

## Tree B: Extended MPCC unavailable

```text
One of:
  invalid/unreachable corridor
  incompatible warm start
  numerical conditioning
  stale/mismatched async context
  solver convergence failure                                      [Hypotheses]
        |
        v
extended problem build or solve rejected                           [Detection]
  -> circuit breaker / requalification                             [Mask]
  -> 3-state progress or legacy MPC executes                       [Mask + I-05 violation]
  -> later wall/target check or command output fails               [Symptom]
```

The earliest cause inside the hypothesis set is **Unknown**. Slice-specific replay must capture the
problem fingerprint, matrices/scales, warm-start key, corridor reachability and solver telemetry
before changing OSQP parameters.

## Tree C: Candidate certificate versus executed trajectory

```text
tactical candidate / DP prefix accepted
  -> solver builds a related but not immutable-identical problem   [I-02 detection gap]
  -> extended primal converted to legacy output representation
  -> executed prediction reconstructed/postprocessed
  -> physical wall validation checks the executed representation   [Detection]
       -> valid: publish or retain
       -> invalid/requalifying: wall hold/replan                    [Mask/Recovery behavior]
  -> non-acceleration or -3.0 m/s2 is visible                       [Symptom]
```

Recent wall-contract changes reduce this gap, but one end-to-end fingerprint still does not prove
that candidate, problem, prediction, certificate and command are one object. The correct fix is not
to remove physical validation; it is to move the validation result into the selected solution
certificate.

## Tree D: Dynamic Escape fresh-result gap

```text
same encounter remains active
  -> tactical candidates update and side/branch may change
  -> no fresh candidate for the current cycle
  -> retained solution identity compared with current attempt/target/side
       -> match: retained same-solution stages continue
       -> mismatch: execution lease cannot adopt retained solution
  -> outgoing prediction is evaluated during replacement handoff
  -> wall admission requalification hold                            [Mask]
  -> fresh solution arrives next cycle and acceleration resumes     [Symptom/churn]
```

Commit `dc51093` repaired the specific conflation of fresh-candidate availability and persistent
execution ownership. Run `20260822-105057` still shows side changes and a
`retained-identity-mismatch` around decisions 1386-1393. The remaining earliest violation is
**Unknown**: it may be branch selection churn, incomplete context identity, or a legitimate hard
safety preemption. Replay R06/R08 must decide before another handoff patch is allowed.

## Tree E: Recovery after normal-control failure

```text
normal control loses progress or contacts wall/vehicle
  -> solver/wall/fallback/direct-control output changes
  -> stopped/no-progress detector confirms an episode
  -> Recovery takes command/gear authority                          [Recovery behavior]
  -> reverse/rejoin succeeds or SafeStop remains                    [Outcome]
```

Recovery is not the root cause of the original wall departure or solver failure. It remains outside
the canonical forward MPCC, but its entry trace must retain the upstream normal solution/failure
fingerprint (I-10).

## Minimal causal cut sets to test

1. **Formulation cut set**: intent/activation change + cross-formulation fallback. Removing either
   should eliminate MPC/MPCC command churn.
2. **Identity cut set**: incomplete context fingerprint + asynchronous/retained adoption. Full
   fingerprint comparison should reject the stale result before authority changes.
3. **Wall cut set**: non-identical planned/executed representation + late physical validation.
   Certifying the actual executable solution should move rejection upstream without weakening walls.
4. **Recovery cut set**: upstream normal failure + lost causal identity. Carrying the fingerprint will
   not prevent the failure, but will stop Recovery from hiding its origin.
