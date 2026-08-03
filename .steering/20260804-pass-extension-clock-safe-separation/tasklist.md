# Tasklist

## Investigation

- [x] `output/20260804-001441`のPass/Recovery理由を集計する
- [x] ROS時刻とsteady clockの混在箇所を特定する
- [x] SafeSeparationの即時Recovery条件を特定する

## Implementation

- [x] Pass延長計画のcommit時刻をROS時刻基準へ正規化する
- [x] SafeSeparation前方クリア連続確認状態を追加する
- [x] 設定読込・起動ログ・`config.yaml`を更新する
- [x] 単体テストを追加・更新する

## Verification

- [x] `test_v2x_overtake_core`（329件、失敗0）
- [x] `git diff --check`
- [x] `make autoware-build`（25 packages successful）
- [ ] `make dev2` 6周（ユーザー確認）

## Dynamic acceptance metrics

- `atomic commit: prediction expired`回数
- `ShiftOut -> Pass`から`Pass -> Recovery`までの時間
- `Pass -> Return` / `Pass -> Recovery`回数
- SafeSeparation開始、前方クリア連続確認、timeout回数
- Reverse要求、55秒超過ラップ、壁接触

## Verification notes

- `make autoware-build`: 成功。stderrは既存のsetuptools `setup.py install`非推奨警告のみ。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R
  test_v2x_overtake_core --output-on-failure`: 成功。
- `test_v2x_overtake_core.gtest.xml`: tests=329, failures=0, errors=0。
- `colcon test-result --verbose`は全保存結果829件で失敗0。無関係な古い
  `build/joycon_contract_guard/package.xml`欠損を読み飛ばす診断を出したが、終了コード0。
