# Results

## Static verification

- `make autoware-build`
  - 25 packages成功。
  - `multi_purpose_mpc_ros`を含む全packageが完了。
  - stderrは既存のsetuptools deprecation warningのみ。
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 28/28 test targets成功。
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros/test_results --verbose`
  - 1238 tests、0 errors、0 failures、0 skipped。
- `git diff --check`
  - 問題なし。

## Implemented behavior

- baselineの `d'(s)` とreference curvatureから各stageの車体headingを算出する。
- headingを反映した2.0 m x 1.45 m footprintで、baseline近傍の連結wall-free intervalを求める。
- preferred reserveとhard reserveを分離し、optimizer、事後execution bounds、下流MPCで共有する。
- 全profileのcoupled footprint validationは最終guardとして維持する。
- RecoveryRetentionから再開したMissionの将来horizonがhard infeasibleなら、同じMissionを
  Recoveryへ再投入せず終了し、失敗sideを短時間blockする。
- active execution中の将来physical missは、実接触などのhard faultがなければ
  last-feasible trajectoryまたはdynamic waitを先に試す。

## Dynamic acceptance criteria

`make dev2`で `output/20260818-081035` と同等時間を走行し、次を確認する。

- `optimized horizon failed physical revalidation`: 9回未満。
- 同一Missionの `Recovery -> FollowPrepare -> Recovery` 反復が発生しない。
- `retained pass mission horizon still infeasible` 後に同一Mission generationへ戻らない。
- `Pass -> Return -> Idle` 成功を維持する。
- physical envelope探索追加によりcontrol callback overrunが増えない。
- 実接触、unknown/out-of-map、EmergencyBrakeのhard guardが維持される。
