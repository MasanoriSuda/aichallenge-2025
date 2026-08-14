# Results

## 実装結果

- DP実行profileをtarget予測区間だけでなくGapPlannerの全horizonへ延長した。
- target非占有区間ではrobust wall planning clearanceを適用した。
- active ShiftOut/Pass中に、同一target・同一sideかつ現周期で成立したfresh DP列だけを
  0.10秒間隔でatomic更新するrolling refreshを追加した。
- 更新不可の周期は直前の可解DP列を保持し、Mission generation、Pass進捗、target lockは保持する。
- freeze、refresh、実行coverage・残距離・refresh回数をruntime logへ追加した。

## 静的検証

- `git diff --check`: 成功
- Docker内`colcon build --packages-select multi_purpose_mpc_ros --symlink-install`: 成功
- Docker内`colcon test --packages-select multi_purpose_mpc_ros`: 25 test targets成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros/test_results --verbose`:
  1069 tests、0 errors、0 failures、0 skipped

ビルド時のstderrはROS 2 Kilted向けheader install推奨とsetuptoolsの既存deprecation warningのみ。

## 未完了

動的効果は試走待ち。`tasklist.md`の動的確認項目で判定する。
