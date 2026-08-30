# Results

## Root-cause conclusion

`output/20260830-152829`のfrozen Return failureでは、historical artifactとの差
`-1.541660 m`が`1.5 m` gateを超えた時点でproduction Aだけがrejectした。同一requestを
現時刻から再構築したshadow Bはwall、全peer、terminal Stopを含めて
`accepted/proof=1`だった。

したがって根因は物理的不可能、candidate generation、single-SQPではなく、historical
Mission/artifact clockをcurrent-world authorityの前提にしたlifecycle defectである。

## Implemented change

- `Reason::ProgressLiftRejected`を削除した。
- `evaluate_impl(..., enforce_progress_continuity)`によるproduction/shadow二重評価を削除した。
- progress差をreject条件から`progress_rebased` provenanceへ変更した。
- rebase時も、現在pose/speed/serialized steeringを起点にexact continuation、壁、全peer、
  terminal Stopを完全に証明する。
- 証明成功時だけstateless current-world Bundleとしてauthorityを与え、publication ledgerを
  現在のBundle sourceへreanchorする。
- パラメータ、clearance、solver、fallback、Mission timingは変更していない。

## Static verification

- `make autoware-build`: 成功、25 packages。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功。
- `colcon test-result --verbose`: `2241 tests, 0 errors, 0 failures, 0 skipped`。
- source-contract testは旧reason、旧二重評価、旧authority branchの不在を固定した。
- unit testはprogress差がtoleranceを超えても、完全証明成功時だけ
  `progress_rebased=true`のproofを返すことを確認した。

`colcon test-result`には既存の`joycon_contract_guard/package.xml`欠損に関するskip診断が
出るが、テストsummaryは全合格であり今回変更とは無関係である。

## Dynamic verification

### Invalid startup

`output/20260830-154328`は2台とも`spawned`から進まずodometryが出なかったため、制御評価に
使用せず破棄した。

### Valid dev2 run

`output/20260830-154652`では`grounded -> ready`後に走行した。

- `Idle -> ShiftOut`: 4回
- `ShiftOut -> Pass`: 2回
- `current_world_rebase:1`: 1回
- `progress-lift-rejected`: 0回

decision 1698では、historical artifactとの差が`-4.345142 m`、旧toleranceが`1.5 m`で
あったにもかかわらず、現時刻Bundleは`current=accepted`となった。旧gateならこの差だけで
authorityを失ったが、新実装では差はprovenanceに留まり、wall/peer/terminal proofが
publication可否を所有した。進捗差を直接原因とするEmergency Stopは再発しなかった。

## Separated remaining failure

このrunの追い越し全体は完遂していない。観測された失敗はprogress rebaseより上流で、
次の一貫した系列だった。

1. `ShiftOut -> Pass`
2. `Pass entry physical gate has no valid current-side prefix`または
   `Pass entry physical wall gate unresolved`
3. `Pass -> FollowPrepare`
4. cross-side replacementもcommit時にreject
5. `static wall physical footprint infeasible`からRecovery

従って今回Sliceはhistorical progress lifecycle defectを解消したが、Pass採用時とruntime
wall/current-side prefixの可行性不整合は残る。Return再現・完遂が得られなかった理由を
progress authorityへ戻して解釈してはならない。

## Next root-cause slice

次はclearanceやwall marginを変更せず、以下を同じworld fingerprintで比較する。

- Mission採用時のPass wall certificate
- runtime Pass-entry physical wall gate
- current-side prefix生成時のcourse frame、footprint、progress origin
- cross-side replacement commit時の同じ証明

採用時は成立し、実行時だけ不成立になる最初のstate/coordinate/provenance差を特定してから
修正する。
