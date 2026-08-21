# Tasklist

- [x] 直近ログと現行candidate lifecycleを照合する
- [x] 変更範囲、非対象、受入条件を文書化する
- [x] 将来リスクとbranch選択をpure functionへ分離する
- [x] speculative GapPlannerとlive continuity commitを分離する
- [x] DynamicEscape入口へ予防的反対側評価を接続する
- [x] decision traceへ予測・評価・採用理由を追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 次回試走の確認項目を記録する
- [x] 変更をコミットする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32 targets成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1455 tests、0 errors、0 failures、0 skipped
- ROS topic、message、launch、評価成果物契約の変更なし

## 次回試走の確認項目

- `Overtake decision trace: stage=planning`で、主候補の
  `forecast=1/1`と`forecast_reason`がhard failureより前に出ること。
- `alternate_trigger=corridor-reserve|wall-margin-escape|tracking-wall-contract`
  のとき、反対側候補が同じattempt IDで記録されること。
- `branch_selection=lower-risk-tier|corridor-reserve-advantage`なら
  `proactive_alternate=1`、`alternate_selected=1`となること。
- 反対側が`forced-side-empty`なら、主候補がusableな間は失効させず、ログ上
  `branch_selection=alternate-unusable`として理由が残ること。
- alternate不採用後の次周期に、live primary sideがspeculative sideへ勝手に反転しないこと。
- DynamicEscape開始からwall stopまでの件数、hard failure後のalternate初回評価件数、
  Follow/停止へ落ちる前の反対側採用件数を比較すること。
