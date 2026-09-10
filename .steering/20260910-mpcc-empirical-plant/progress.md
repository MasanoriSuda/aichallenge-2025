# Shared-model migration evidence (in progress)

Baseline HEAD remains6b0f1880. All new build/tests below apply to the working
nine-state migration, not to that baseline commit. No new-model simulator run yet.

- Native/public input comparison: `output/20260910-empirical-plant-public-inputs-r2/report.json`.
  Public ROS coverage gates every scoring horizon; dev2 restart lies outside that old bag.
  Single holdout35..59.54s/488anchors:1s position MAE0.19366130196m/max0.67590242266m.
  The original fixed Python/native comparison8212values differs at most4.50e-15.
- Build r3:26packages pass; package r1:2386records/127failures.
- Build r4:26packages pass; package r2:2386records/32failure records,27failing test cases.
  Failures include old fixed state counts/missing model hashes, old wire=net fixtures,
  short synthetic Stop domains and remaining free-control Stop behavior.
- Build r5:26packages pass (4min50s), no C++warnings/errors. Logs:
  `/tmp/mpcc-nine-state-build-r5.log`, package r3 currently running.
- Fixed-constraint counterexample search:
  `output/20260910-nine-state-fixture-audit-r1/{shadow.cpp,run.py}` and
  `/tmp/mpcc-nine-state-fixture-audit-r1.log`. The valid9state
  `(0,0,-.5,2,.04,-.3,-.2,1,0)` rejects the direct solve, solves the reachable
  bridge but fails exact physical proof; two-step audit rejects after one solve.
  This replaces the old fixture whose tire angle-.44rad exceeds the native tire limit.

New immutable public observation provenance records source pose/body, each sensor
source stamp, now/control clocks, nominal per-channel delays and serialized history.
Snapshot roundtrip and epoch/history mutation are covered by a regression. Stop tire
error diagnostics now compare physical tire to physical tire, independently of desired
steering. Complete-rest candidate feasibility now uses the shared body model instead
of the remaining wire=net reachability shortcut. Terminal fixed-zero speed also fixes
lateral velocity/yaw rate to zero in the QP. Integration count uses the common helper
for fixed5ms proof mapping; general configured-substep consistency still to audit.

No margin, tolerance, retry allowance, solver budget or production objective was relaxed.
M3 regressions and the remaining M1-M6 acceptance work remain open; no completion claim.

## Rest-mode comparison and proof repair

Buildr6:26packages pass (4min51s). Package r4:2387records/5failure records:
2related free-Stop cases plus1old seven-state comment assertion. All earlier
retained, cross-peer and false-zero-endpoint regressions now pass.

Frozen current-world free-Stop:
`output/20260910-nine-state-mode-comparison-r1/snapshot/000000000009-52db8fa2e4c33e02-cruise-side-neutral-initial-nine-state-rest-mode/snapshot.yaml`,
fingerprint5970523660795198978. Final rest-mode tangent's velocity A/B/offset all0;
QP speed0 but physical1.61453m/s. Direct relinearization and replay-state
relinearization both certify after1correction under the same hard scene. Four
independent held-speed→brake laws atstages12..15 also pass the same nonlinear
wall/peer/terminal proof. No infeasibility claim. Logs `/tmp/mpcc-nine-state-mode-comparison-r1.log`.

Producer correction: seal `terminal_body_rest_required` in full execution artifacts,
check exact u/vy/yaw-rate rest in the shared physical adapter, and include this
failure in the existing physical-proof SQP correction loop. Its maximum3,
all solver settings and physical tolerances are unchanged. Nonlinear max-braking
oracles must also preserve the sealed wire law. A new regression refuses a
QP zero endpoint with physically moving input replay.

Remaining fixture snapshot367272d907e582e9 is preserved at `output/20260910-nine-state-rest-endpoint-r1/snapshot.yaml`.
Native replay ends u1.6440739894e-8,vy7.9191343076e-7,r6.4552875201e-6 with terminal
positive wire1.7609098316e-6. This is not the nominal rest mode. The free-Stop
producer now declares its model-derived terminal wire<=0 condition before solver
insetting. It does not round/clamp a solved command or waive exact rest.
Build/test after that correction is still pending.

## Frozen candidate verification

Buildr8 passes26packages; package r6 passes2387records with0errors/failures/skips.
Repository applicable pre-commit hooks pass. Native/public comparisonr3 uses
nativer2,8212values maximum4.4964e-15;1s holdout position MAE0.193661302m,
maximum0.675902423m. `evidence.json` binds sources, patch and preserved logs.
Earlier pending build/test statements above are historical. Fresh runtime follows.
