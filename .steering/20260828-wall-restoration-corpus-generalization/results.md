# Results

## Scope

This Slice changed only the audit arm
`evaluate_wall_feasibility_restoration_audit()`. Production `evaluate()`,
authority, physical margins, OSQP settings, Mission state and fallback remain
unchanged.

The Phase-I projection now restores every pose state box owned by the
post-hoc physical wall bucket at states 1..N:

- lateral;
- lag;
- heading;
- progress.

It retains affine dynamics, actuator and velocity bounds, progress-wall rows,
swept-wall rows and dynamic-obstacle rows. The relaxed solution remains only a
non-certifiable tangent. A fresh full physical refinement is mandatory before
an artifact can be considered.

Failure capture was also corrected so a
`wall-refinement-restored-solve-rejected` snapshot records the actual Phase-I
QP and warm start rather than the original full-refinement QP.

## Frozen corpus

Independent HiGHS LP classification of the 17 recorded exact QPs found:

- 13 affine-infeasible QPs;
- four affine-feasible QPs.

This disproves a universal solver-only explanation. The bounded generalized
restoration replay then produced the following classifications.

| decision | intent | generalized restoration | independent exact result | classification |
|---:|---|---|---|---|
| 1161 | ShiftOut | Phase-I rejected | Phase-I QP is affine-infeasible; removing explicit wall rows restores feasibility | selected physical wall envelope is incompatible with the reachable affine continuation |
| 1566 | Pass | final production QP solved | exact current-world wall proof rejects by about 0.001 m at substage 236 | model/certificate mismatch at the post-refinement wall boundary |
| 2473 | ShiftOut | three Phase-I SQPs; final OSQP reaches 4000 iterations | HiGHS solves the QP, but exact current-world wall proof rejects at substage 244 | backend mismatch followed by a wall model/certificate mismatch |
| 3931 | Pass | three Phase-I SQPs; final OSQP reaches 4000 iterations | HiGHS solution passes exact QP, nonlinear trajectory, swept wall, all-obstacle dynamic and successor proof | genuine feasible-QP backend mismatch |
| 4909 | ShiftOut | three Phase-I SQPs; final OSQP reaches 4000 iterations | HiGHS solution fails dynamic proof against `d2` by 0.000379 m | backend mismatch followed by a dynamic model/certificate mismatch |

The independent HiGHS final-QP violations for decisions 2473, 3931 and 4909
were `8.99e-15`, `2.58e-14` and `3.85e-14` respectively. These prove the
serialized affine rows are feasible; they do not by themselves prove a safe
trajectory.

Decision 3931 is the only generalized restoration result in this corpus that
forms a complete audit-only `ManeuverBundle`. Decisions 1566, 2473 and 4909
correctly remain unpublished because the unchanged physical proof chain
rejects them.

## Root cause

The frozen failures are not one defect:

1. **Candidate/wall-envelope construction defect**: decision 1161 has no
   affine continuation inside the selected explicit wall envelope, even after
   every refinement-owned pose bucket is restored.
2. **Feasibility-restoration defect**: decisions 2473, 3931 and 4909 need a
   new tangent before the full wall QP has a non-empty feasible set.
3. **QP backend mismatch**: the rebuilt full QPs are feasible, but production
   OSQP stalls on dual convergence within the existing contract.
4. **Model/certificate boundary mismatch**: some affine solutions still fail
   the stricter current-world swept-wall or dynamic-obstacle proof.

The restoration generalization is therefore useful diagnostic structure, not
a production fallback. Promoting it alone would merely move failures from the
solver boundary to the physical certificate boundary.

## Existing patch relationship

- More Mission resume rules, side leases or grace periods do not create an
  affine-feasible wall continuation for decision 1161.
- Increasing OSQP iterations or accepting its last iterate would not repair
  decisions that fail exact wall/dynamic proof.
- Relaxing wall or opponent clearance would hide the certificate mismatch and
  was not attempted.
- A backend replacement alone repairs only the decision-3931 class; it is not
  a corpus-wide solution.

## Production recommendation

Do not promote generalized restoration to production in this Slice.

The next structural work should keep the same immutable-world replay and split
into two bounded questions:

1. move the exact current-world wall certificate into the same
   refinement/relinearization ownership boundary, so a candidate rejected by
   that certificate can request a new tangent rather than appear solved
   internally and fail downstream;
2. separately evaluate a production-packaged QP backend/globalized SQP only
   on QPs independently proved feasible, retaining the complete exact proof
   chain and last-published certified artifact semantics.

Decision 1161 must return to candidate/homotopy generation. It may not enter a
solver fallback because its selected wall envelope is itself infeasible.

## Verification

- generalized audit replay completed for decisions 1161, 1566, 2473, 3931
  and 4909;
- independent HiGHS QP solve and the unchanged C++ exact proof chain completed
  for every generalized final QP that was affine-feasible;
- `make autoware-build`: 25 packages passed;
- `colcon test --packages-select multi_purpose_mpc_ros`: 52/52 CTest
  targets, 2056 tests, zero errors and zero failures;
- production authority remained unchanged;
- the only test-result warning is the pre-existing stale generated
  `build/joycon_contract_guard/package.xml` lookup.

## Remaining risks

- The corpus is evidence from recorded failures, not a full race acceptance
  matrix.
- The 1 mm-class wall/dynamic discrepancies must not be dismissed as numeric
  noise until the model and exact-certificate coordinate/time ownership are
  reconciled.
- HiGHS is currently an offline audit dependency and has no approved
  production packaging or real-time contract in this repository.
