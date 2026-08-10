# Tasklist

- [x] 最新実走のtail解除とprediction graceを照合する
- [x] 要求・設計を文書化する
- [x] rear-clear tailを既存prediction-only graceへ接続する
- [x] grace由来tailの観測性を追加する
- [x] core回帰テストを追加する
- [x] 対象package testを実行する
- [x] `make autoware-build`を実行する

## Definition of Done

- tail候補でprediction-only graceがactiveならlocal limitからAbortしない。
- grace上限超過、corridor block、hard guard failureではtailを維持しない。
- physical tailの既存挙動を維持する。
- 対象単体テストとpackage buildが成功する。

## 実走確認項目

- `SideBySide rear-clear tail active`後の短いprediction lossでRecoveryへ落ちない。
- grace内にprediction sweepが復旧するか、rear-clear後`Pass -> Return -> Idle`へ進む。
- grace超過またはconfirmed current overlapでは従来どおり中断する。
- wall contact、solver failure、Reverseが増えていない。

## 静的検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  943 tests、0 errors、0 failures、0 skipped
