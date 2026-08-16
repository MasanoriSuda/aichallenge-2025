# Tasklist

- [x] 最新走行と既存 hold 経路を照合する
- [x] receding-horizon hard failure を型付きにする
- [x] target/wall interval conflict を target-only と分類する
- [x] incomplete ShiftOut 用の current-lateral prefix と短時間 budget を追加する
- [x] local / cloud config と起動ログを更新する
- [x] core 単体テストを追加する
- [x] package test/build を実行する
- [x] 差分をレビューしてコミットする

## Static verification

- `make autoware-build`: success (25 packages)
- `colcon test --packages-select multi_purpose_mpc_ros`: 27/27 test targets passed
- `colcon test-result --verbose`: 1235 tests, 0 errors, 0 failures, 0 skipped
  - 既存の `build/joycon_contract_guard/package.xml` 欠損に関する stale artifact warning は出たが、今回の対象テスト結果には影響なし。
- `git diff --check`: passed

## Review

- Critical / Warning finding: なし
- topic、service、Domain、launch、result schema の変更なし
- wall / current-body / emergency / solver / forbidden-waypoint の hard fault は hold 条件より優先される
- 動的効果（Recovery削減と接触増加の有無）は次回 `make dev2` 走行で確認する

## Dynamic verification (user run)

- [ ] `physical target separation conflicts with wall bounds` 直後に Recovery せず hold / replan へ入る
- [ ] ShiftOut hold が 0.35 s / 2.0 m を超えて延長されない
- [ ] wall / current-body hard fault は hold によって隠れない
- [ ] `Pass -> Return -> Idle` 完遂率と 60 s 超ラップが改善する
