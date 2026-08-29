# Results: independent nonlinear feasibility oracle

## Frozen boundary

- production baseline: `6884807f`;
- ShiftOut: sequence 1266, fingerprint `145d1159f38a6ea9`;
- Cruise: sequence 601, fingerprint `92df63438ac7ffd7`;
- production authority, solver policy, clearance, timeout, lease, fallback and
  runtime configuration were unchanged.

## ShiftOut: single-SQP/model limitation

The deterministic CasADi/IPOPT oracle integrated the exact seven-state
nonlinear dynamics at at most 10 ms per substep.  All 12 starts converged with
zero recomputed retained-physical violation.  The best generated primal then
passed the unchanged C++ exact trajectory, wall, timed dynamic-obstacle,
terminal-successor and Stop-suffix proof chain:

- terminal progress: `15.5011 m`;
- terminal velocity: `6.12149 m/s`;
- minimum lateral reserve: `1.40457 m`;
- external exact proof: accepted.

The same physical world is therefore feasible.  The affine/dense OSQP arms
which reach 4000 iterations do not prove physical infeasibility; this frozen
failure belongs to the single-SQP/convexification and numerical formulation
boundary.

## Cruise: candidate-generation defect

The captured neutral Cruise candidate is genuinely inconsistent as written.
Twelve deterministic nonlinear starts converged to the same minimum required
slack, approximately `0.506597 m`.  Its first owning conflict is between:

- `dynamic[5].lateral:upper`, and
- the dense/progress-dependent wall lower envelope around stages 19--20.

That does not mean the scene is physically blocked.  The existing architecture
comparison previously emitted `unsupported-intent` for every stateless Cruise
arm and therefore compared no alternative.  After adding an observation-only
Cruise comparison, the same immutable world produced:

| Arm | Outcome | Terminal progress | Terminal velocity | Lateral reserve |
|---|---|---:|---:|---:|
| captured neutral branch | solver rejected | N/A | N/A | N/A |
| explicit positive side | exact proofs accepted | 9.41925 m | 5.90443 m/s | 1.21804 m |
| explicit negative side | exact proofs accepted | 9.47662 m | 5.90443 m/s | 1.45082 m |

The first violated invariant is therefore not wall feasibility or OSQP
conditioning.  Neutral Cruise converted one obstacle-free wall witness into a
single automatic behind-to-side schedule using scalar per-stage box tests.
Those tests do not prove the coupled dynamics, progress-dependent wall
envelope and obstacle disjunction together.  Both complete current-world side
candidates pass the unchanged full proof chain.

## Causal chain

1. `mpcc_rate_resolved_dynamic_obstacle.cpp` receives neutral `side_sign=0`.
2. The automatic branch chooses one side from initial relative lateral sign
   and scalar state-box feasibility.
3. It emits a behind/diagonal/lateral schedule before complete coupled
   feasibility is known.
4. The later lateral upper rows conflict with the physical wall lower
   envelope by about 0.51 m.
5. OSQP reaches maximum iterations; solver failure is the detector, not the
   producer.
6. The old comparison also rejected Cruise alternatives before construction,
   masking the candidate-generation defect as apparent general infeasibility.

## Existing patch relationship

Internal equilibration, dense wall rows and extra local wall cuts cannot fix a
candidate whose chosen obstacle disjunction conflicts with its wall envelope.
They remain useful audit evidence but are not a production remedy.  No new
retry, tolerance, fallback, grace period or clearance change is justified.

## Next architecture action

Normal Cruise/Follow dynamic-obstacle handling must stop treating the single
automatic branch as the only candidate.  The next production Slice should
atomically replace that producer with the bounded current-world side
population under the existing seven-state SQP and unchanged exact proofs.
The selected artifact remains neutral Cruise/Follow intent; this must not
create Overtake Mission authority or a second publisher.  The old automatic
neutral-side producer must become unreachable in the same Slice.

ShiftOut needs a separate successive-convexification/structure-exploiting
Slice.  It must not be mixed with the Cruise producer repair.

## Verification completed so far

- `python3 -m py_compile constrained_nonlinear_oracle.py`: passed;
- focused stateless-maneuver tests: 2/2 passed;
- focused architecture-comparison tests: 2/2 passed;
- `make autoware-build`: 25 packages passed;
- full package regression: 54/54 CTest targets passed;
- experiment-registry validation: passed;
- `git diff --check`: passed;
- ShiftOut nonlinear external primal: unchanged exact proofs accepted;
- Cruise captured/positive/negative comparison: completed.
