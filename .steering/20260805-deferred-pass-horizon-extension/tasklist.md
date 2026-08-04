# Task list

- [x] 最新runのPass失敗経路を集計する
- [x] 要求・設計を作成する
- [x] rear-clear replan window純粋判定を実装する
- [x] Pass horizon callerへ接続する
- [x] 延期ログを追加する
- [x] 境界回帰テストを追加する
- [x] build/package testを実行する
- [x] 結果を記録する

## 動的確認項目

- `Pass rear-clear extension deferred`の発生数
- Pass開始直後0.5秒以内のSafeSeparation発生数
- `rolling outer transition accepted`件数
- `Pass -> Return` / `Pass -> Recovery`件数
- SafeSeparation trigger/failure内訳

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 targets）
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  843 tests、failure/error/skip 0
- `test_v2x_overtake_core`: 346 tests、failure/error 0
- `git diff --check`: 成功
- `make dev2`は未実施。上記動的確認項目を次走で照合する。
