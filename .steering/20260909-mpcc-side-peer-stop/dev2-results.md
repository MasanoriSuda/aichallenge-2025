# Changed-input dev2 result: rejected

Run `output/20260909-side-peer-stop-dev2-r1`, baseline b5308839 plus sealed
patch SHA256 `3b00a480d9669c786fb7acd428d693491a1eb41facb1b6ca60f36f9c70d046af`.
Full build: 25 packages. MPCC: all 60 CTest groups / 2346 colcon records pass,
zero errors, failures or skips. The setup.py deprecation warning is unrelated.
The optional host pytest command failed collection because localization_scope
was unavailable there; the supported container package run includes all 104
source-contract tests and passes. No host collection failure is counted as a pass.

The native current zero-objective Stop still certifies in final-binary replay of
world396, approximately 7.97 ms, candidate fingerprint 5013267340109276277.
The separate historical-axis arm remains rejected; all outcomes are retained.

## First integrated failure

Domain 2, Ready, decision 978, wall-clock 1788895054.539998030, speed 1.58 m/s:
Emergency after current-world reproof of a published certified Stop fails its
wall reserve. This is not proof of an actual vehicle-wall contact.

- At decision 829, source 297 was adopted as a certified Cruise-origin Stop at
  rest. Decision 862 explicitly reports `source-cruise-executed-retained`, then
  normal acceleration resumes. The seven-intent candidate path is exercised.
- At decision 976, ordinary normal authority loses its terminal contingency.
  Current-world Stop source 442 / decision 972 is accepted and selected.
- At decision 978, the published Stop successor reports `static-path-blocked`.
  Delay prefix and publisher interval are clear; continuation and both terminal
  references fail wall proof. Peer proof is clear (approximately 0.929 m).
  Control-pose displacement from the old trajectory is 0.033732 m, yaw difference
  0.002601 rad; current/control-origin/old predicted speed are
  1.577263 / 1.639423 / 1.117173 m/s. These are observations, not root-cause proof.
- The external Stop selector correctly emits Emergency after proof failure.
  Its `published_stop_retained` label also suppresses the final authority-loss
  recorder, leaving the actual source 442 Stop artifact absent. The earlier
  decision 976 snapshot and published normal source 962 are present. Re-solving
  either cannot recreate the missing executed Stop and must not be called a
  replay of decision 978.

No laps were completed before termination and neither Domain result-details
JSON exists. Finish/penalty acceptance is unavailable. The monitoring decision
was saved at wall-clock 1788895059.8295183 (filesystem mtime, approximate teardown
boundary). Later stale-odometry / lost-peer events occur during sequential
container shutdown and are retained separately from the first failure. The full
log's Domain 2 callback overrun likewise occurs after this teardown boundary.
Before that boundary, reported callback cycles/max/overruns are Domain 1:
1176 / 18.840 ms / 0; Domain 2: 1175 / 21.028 ms / 0.

All containers are down. Both user-owned JSON files were restored byte-for-byte;
the generated replacement summary is retained under the run directory and the
restoration receipt is `artifact-restoration.json`.

## Next evidence

The first saved normal QP, Domain 2 source 381, fails all four cold/warm and
normalized/equilibrated arms at 4000 iterations. Independent LP minimization
finds minimum normalized row violation 1.111762. An exact rational dual check,
with all multipliers nonpositive, exact state stationarity and bounded input
residual, proves the encoded affine rows cannot meet even their existing physical
tolerances (positive contradiction margin 0.11176197374636564). This certificate
is about that convex branch only. It does not prove the physical scene infeasible.
Increasing iterations or changing initialization cannot repair those rows.
The certificate and all four numerical arms are under
`output/20260909-side-peer-dev2-normal-qp/`.

Do not rerun this unchanged acceptance trial or relax wall reserve. Compare the
convex branch's geometry against feasible alternative trajectories. Fix
only the observation predicate that discards a proved Stop's authority loss,
then obtain the missing source/artifact/current-world tuple in a bounded
diagnostic run. Compare the commanded/observed motion and independent rollout
before another physical or control-authority repair. Stop continuity and the
full single/dev2/dev3/dev4/submission campaign remain open.
