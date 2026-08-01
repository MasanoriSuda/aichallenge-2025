# Tasklist

- [x] 最新runのFSM滞在時間と失敗理由を確認
- [x] 既存未コミット差分とinterface契約を確認
- [x] 要件・設計・非対象範囲を定義
- [x] 横経路目標policyをcoreへ抽出
- [x] Recovery後mission保持policyをcoreへ抽出
- [x] committed-pass geometryをcoreへ抽出
- [x] closing reserve policyをcoreへ抽出
- [x] controllerをpolicy呼び出しへ置換
- [x] 単体テストを追加
- [x] package build/testを実行
- [x] 差分レビューと次の性能修正境界を確認

## 検証結果

- `make autoware-build`: 成功（25 packages）。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25）。
- `colcon test-result --verbose`: 789 tests、0 errors、0 failures、0 skipped。
  - workspace内の既存`build/joycon_contract_guard/package.xml`欠損警告は継続するが、
    対象packageのtest failureはない。
- `test_v2x_overtake_core`: 265 tests成功。
- `git diff --check`: 成功。

## 次ステアリングへ残す性能修正

- `FollowPrepare`の時間・走行距離上限とmission再選択。
- ShiftOut/Pass/Returnを一体で検査する全区間candidate path。
- 採用後のcorridor goal固定と相手横揺れからの分離。
- rear clearance確定後の同一target再捕捉禁止。
