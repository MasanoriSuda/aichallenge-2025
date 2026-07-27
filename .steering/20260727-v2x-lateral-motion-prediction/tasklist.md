# Task list

- [x] 既存runと現行予測経路を確認する
- [x] 変更範囲とA/B条件を固定する
- [x] course lateral prediction helperを追加する
- [x] V2X前回・現在位置の2点投影をgap plannerへ接続する
- [x] 設定と起動時ログを追加する
- [x] 単体テストを追加・実行する
- [x] 仕様書と本tasklistを更新する

## Definition of Done

- course-progress縦予測を無効のまま横方向予測だけ有効化できる
- 無効・異常入力では既存Cartesian予測へfallbackする
- deadbandと最大横速度の境界を単体テストで確認できる
- 対象packageがbuildできる

## 検証結果

- `g++ -std=c++17 -Wall -Wextra -Werror ... -fsyntax-only`: 成功
- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 24/24 test成功
  （test result全体では674 tests、0 failures）
- カーブ誤検知テスト追加後の`test_v2x_overtake_core`: 188 tests成功
- 実走A/Bは未実施。A/B条件は`design.md`に記載した。
