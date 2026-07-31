# Tasklist

- [x] 最新ログと現行 speed ownership を確認
- [x] 性能変更の範囲と非変更条件を定義
- [x] committed speed floor を物理横離隔済み ShiftOut へ拡張
- [x] ShiftOut floor の診断を追加
- [x] Pass 条件と安全条件の回帰テストを追加
- [x] 対象 package をビルド
- [x] 対象 package のテストを実行
- [x] 差分レビューと実走確認項目を整理

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 223/223 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets 成功
- `git diff --check`: 問題なし

## 実走確認

- 低速・停止 target の ShiftOut で `shift_floor=1`, `v_floor=3.00` になること。
- 同じ周期で `cap_release=0` が維持されること。
- 横離隔成立前、target loss、壁接触、SafetyBrake では `shift_floor=0` になること。
- 前回約 1.5 m/s まで落ちた区間で、不要な crawl と Recovery が減ること。
