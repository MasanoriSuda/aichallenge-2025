# Task list

- [x] 現行のouter-role判定とPass extension経路を確認する
- [x] 要求・設計を作成する
- [x] rolling outer replan policyを純粋関数として実装する
- [x] Pass continuationへ検証済みside transitionを追加する
- [x] config、起動ログ、debugログを追加する
- [x] 単体テストを追加する
- [x] package testとbuildを実行する
- [x] 動的確認項目を記録する

## 動的確認項目

- `continuous outer replan`要求・成功・棄却回数
- side変更時のtarget longitudinal、current/predicted body separation
- side変更後のfront-cap再解除時間
- `outer pass becomes inside before rear-clear`件数
- `ShiftOut -> Pass -> Return`完遂数
- 壁接触、車体接触、SafetyBrake、SafeSeparation、Recovery理由

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  841 tests、failure/error/skip 0
- `test_v2x_overtake_core`: 344 tests、failure/error 0
- `git diff --check`: 成功
- `make dev2`の動的効果確認は未実施。上記「動的確認項目」を次走で照合する。
