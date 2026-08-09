# Tasklist

- [x] 最新ログの反復遷移をコードへ対応付ける
- [x] Mission generation 失効ラッチを追加する
- [x] Dynamic Mission wait で失効 Mission の再開を禁止する
- [x] 未検証の全幅切替候補を初期 admission で拒否する
- [x] 単体テストを追加する
- [x] package build / test を実行する
- [x] 検証結果と残課題を記録する

## Validation

- `make autoware-build`
  - 成功: 25 packages finished
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 成功: 25 test targets
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`
  - 成功: 915 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`
  - 成功

## Runtime verification requested

- 同一 generation について `Mission generation invalidated` の後に
  `committed pass resumed on validated same side` が出ないこと。
- 反対側が完全成立した場合だけ `opponent side PassPlan replaced` が出ること。
- 反対側不成立時は `dynamic Mission wait failed: current Mission generation
  invalidated` から Recovery、Idle へ戻ること。
- 初期候補拒否時の
  `unvalidated_full_track_transition_rejected` 件数を確認すること。

## Remaining work

- 動的効果確認は `make dev2` の実走が必要。
- この修正で再開ループを除去した後、中間速不足が残る区間だけを別実験で扱う。
