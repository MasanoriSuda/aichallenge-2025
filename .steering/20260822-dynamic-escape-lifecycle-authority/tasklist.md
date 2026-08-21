# Tasklist

- [x] 最新走行から入口条件による実行権喪失を特定する
- [x] lifecycleとexecution handoffの責務を文書化する
- [x] attemptを有効な計画要求の正本へ変更する
- [x] target-blockingとfailure replan/backoffを分離する
- [x] attempt切替時のhandoff identityを同期する
- [x] lifecycle決定ログを追加し、正常wall admissionログを削減する
- [x] 状態遷移・回帰単体試験を追加する
- [x] 対象packageをビルド・テストする
- [x] 差分レビュー後にコミットする

## 静的検証結果

- `make autoware-build`相当のDocker Compose build: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 tests成功
- `colcon test-result --verbose`: 1519 tests、error/failure 0
- 変更対象の直接実行: decision trace 9、orchestrator 56、V2X core 808 tests成功
- `git diff --check`: 成功
- ROS 2 topic/service/message、launch entry、評価schemaの変更なし

## 動的確認項目

- 同一targetで入口距離を跨いでもattempt IDとDynamicEscape authorityが継続する
- `target-blocking`でsolver backoffが増えない
- `retained-solution-expired` heartbeatが長時間残らない
- `started/released`回数とwall admissionログ量が前走行より減る
- hard wall fault、EmergencyBrake、Recoveryは従来どおり優先される
