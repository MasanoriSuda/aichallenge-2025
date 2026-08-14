# Tasklist

- [x] 最新走行の失敗経路を確認する
- [x] Pro/GitHub案と現行receding-horizon optimizerの差分を整理する
- [x] DP corridorの型と純粋関数を追加する
- [x] DP corridorの単体テストを追加する
- [x] `SideAssessment`へ時系列corridorとDP結果を保持する
- [x] 固定goal不成立時のprefix-only bridgeを追加する
- [x] MPCC-lite scoreへDP topology costを接続する
- [x] previous DP pathのfresh leaseを追加する
- [x] config / startup log / runtime logを追加する
- [x] package buildを実行する
- [x] core testとpackage testを実行する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 1114 tests / 0 failures
- `V2XOvertakeCoreFrenetDpCorridor.*`: 4 tests / 0 failures（最終ビルドで再実行）
- `colcon test-result`には既存の`joycon_contract_guard/package.xml`欠損警告が出るが、
  test summaryは0 errors / 0 failures

## 動的確認（ユーザー実施）

- [ ] `Frenet DP corridor`が左右候補を継続評価している
- [ ] `prefix_bridge=1`後に即 `planning_unavailable`へ戻らない
- [ ] `Pass -> Return -> Idle`が発生する
- [ ] `physical target separation conflicts with wall bounds`が減る
- [ ] 追い越し接触・壁接触・Recoveryが増えていない
