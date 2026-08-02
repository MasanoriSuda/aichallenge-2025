# Tasklist

- [x] 現行候補探索・mission状態・速度仲裁の接続点を確認する
- [x] closing speed候補生成をROS非依存coreへ追加する
- [x] mission候補へclosing speedを追加し、選択順位を更新する
- [x] controllerのcandidate latticeへclosing speed軸を追加する
- [x] 選択closing speedをOvertakeLine missionへ固定する
- [x] BehaviorおよびOvertakeLine速度参照へmission値を接続する
- [x] deadline成立済みShiftOutのBehavior ownershipを追加する
- [x] debugログへ選択速度とownershipを追加する
- [x] core単体テストを追加する
- [x] package test/buildを実施する
- [x] 動的確認項目を記載する

## 動的確認項目

- `mission candidate selected`の`closing`と`body_clear_t`
- `Overtake minimum-motion entry`の`closing`
- ShiftOut中の`closing`がmission値以下で維持されること
- adaptive/reserveが必要な場合だけmission値より下がること
- ShiftOutからbody-clearまでの時間
- OvertakeからFollow/SafetyBrakeへの遷移回数
- wall/solver Recovery、接触、ラップタイム

## 静的検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功、25 packages
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
  - 25 test targets
  - 770 tests
  - 0 errors / 0 failures / 0 skipped

## 残作業

- `make dev2`による動的効果確認
- 0.8 / 1.4 / 2.0 m/sのどのclosing候補が選択されたかをログで確認
- ShiftOut中の`shift_owner=1`とbody-clear前のFollow復帰回数を確認
