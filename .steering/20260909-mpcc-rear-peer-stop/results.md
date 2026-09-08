# Results

The tangent producer defect is locally repaired. Full MPCC acceptance remains
open. Baseline/rollback is4afa9948. The single and dev2 manifests pin the exact
source patch, binaries, configuration and images; neither is an unchanged-HEAD run.

## Candidate comparison on world1629 / actual published1100

-164 lateral targets/ramps under maximum braking all reject the complete
  terminal dynamic proof. No candidate was promoted; this does not prove
  physical infeasibility.
-Independent state/input tangent selection crosses the immutable course end
  by1.19288046e-8m. Intersecting only the selected tangent speed with the
  transition domain permits normal A to solve. Raw primal, physical model,
  hard bounds and strict frame tolerance remain unchanged. The resulting
  normal horizon certifies, but its generated terminal Stop still rejects
  the rear peer, minimum-0.000656495m.
-Adding the same rear peer to the QP gives8rejected comparisons: normal
  trajectories still reject terminal proof or nonlinear rollout; all four
  maximum-braking Stop methods reject the QP. This change is not promoted.
-Solving controls freely through exact terminal rest gives3accepted and
 1rejected offline methods. Automatic0/3 and StayAhead3 certify wall, full
  peer body and rest; StayAhead0 rejects assembly. Candidate fingerprints are
 7040298697514930029 and8371569746108075121. Automatic0 initially accelerates
  and brakes later, unlike the previous prescribed maximum-braking family.
  These are feasibility witnesses, not executable authority. The report saves
  actual controls but does not preserve a complete publication artifact.

## Native/build/runtime

The new terminal-domain regression fails against the old adapter. The repaired
adapter passes19native cases. `make autoware-build`:25packages. Scoped
`colcon test --packages-select multi_purpose_mpc_ros` and test-result:
60CTestgroups/2364records,0errors/failures/skips. See sealed validation logs.

`20260909-rear-peer-single-r1`:6laps253.328887939s, penalty0, moving normal
authority overrides0.257reported callback windows/10423cycles, maximum12.671ms,
0overruns.10446commands,40.000072Hz, maximum receipt gap33.745289ms,
0gaps above50ms. Source stamps have12duplicates/backward events and one75ms
gap; sensor/clock delivery pauses remain separate unresolved observations.

`20260909-rear-peer-dev2-r1` rejects at D2decision968, wall1788906836.416986602,
speed1.762816m/s. Snapshot4483742628100244385 preserves actual published354
and its clock. Both generated terminal references fail wall proof; final
track-reference wall reject index33. Dynamic clearance remains+0.725731m.
Continuation proves only the publisher interval. A fresh Stop435 cannot join
because steering is unreachable. Independently built published Stop successor
certifies, but its bundle439 fails the subsequent terminal reproof. These are
distinct from world1629's rear-peer rejection and require exact replay before
another production change. No finished race JSON exists.
Each Domain reports365cycles and0callback overruns; maxima21.073/17.800ms.
Same-run topic timing JSON separates source stamps from receipt timing.

User-owned result-summary and safety-gate JSON hashes are restored exactly.
No raw run output or user artifact is committed. Offline accepted witnesses,
rejected methods and integrated failure are all preserved, with no safety
margin, solver tolerance, runtime parameter or Emergency policy adjustment.

## Next boundary

Preserve a complete solved through-rest artifact and test exact publication,
next-cycle and async current-world proofs. Existing retained evaluation always
synthesizes maximum braking even when a complete through-rest suffix already
exists. Establish a failing native replay and a precise terminal contract before
changing that consumer. Retire any replaced normal candidate path in the same
promotion slice. Independent Emergency remains separate. Then repeat the
required integrated campaign, remaining intents, dev3/dev4, gates and submission.
