# Tasklist

- [x] dev2ログと現行予測方式を照合する
- [x] 要件・設計・効果確認指標を記録する
- [x] コース追従予測の純粋関数を追加する
- [x] gap plannerへコース投影とfallbackを統合する
- [x] 設定と起動時ログを追加する
- [x] 単体テストを追加する
- [x] MPC統合仕様を更新する
- [x] 対象packageの単体テストを実行する
- [x] `make autoware-build` を実行する
- [ ] dev2で効果確認する（ユーザー実施）

## 実績

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 24/24 tests成功
- `colcon test-result --verbose`: 671 tests、0 errors、0 failures
  - 別の既存build成果物 `build/joycon_contract_guard/package.xml` が存在しないため
    読み飛ばした通知は出たが、対象packageのテスト結果には影響しない。
- 最終ビルド後の `ctest -R "^test_v2x_overtake_core$"`: 1/1成功

## dev2効果確認

baselineは `output/20260727-085009`。

- 起動ログに `course_progress=1` が出ること
- P1の完全追い越し回数がbaseline 2回から増えるか
- `locked target no longer executable` がbaseline 22回から減るか
- `ShiftOut -> Pass` がbaseline 7回から増えるか
- 壁margin違反、横加速度違反、接触、競走停止が増えないか
- 悪化時は `v2x_prediction_use_course_progress: false` で現行予測へ戻せる
