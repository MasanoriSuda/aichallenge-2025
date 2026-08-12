# Tasklist

- [x] 最新試走ログと現行実装を照合する
- [x] requirements/designを作成する
- [x] escape完了を実移動距離だけで判定する
- [x] 停止予測距離をbrake開始だけに限定する
- [x] rolling stepwise Reverse設定とlatchを追加する
- [x] 0.4 mごとのReverse primitive再計画を追加する
- [x] 単体テストを追加・更新する
- [x] Docker内package build/testを実行する
- [x] 動的確認項目を記録する

## Definition of Done

- 実移動2.0 m + 停止予測2.0 mでも4.0 m escapeは完了しない。
- 同条件ではReverse gearを維持したbrakeになる。
- 停止後に実距離が4.0 m未満ならReverseCreepを再開する。
- 実移動が4.0 mへ達した場合だけDrive/Rejoinへ進む。
- rolling stepwise中は0.4 mごとにgearを変えず候補を再選択する。
- build/testが成功する。

## 動的確認

- `make dev2`
- P1の`maneuver_distance`が4.0 m付近で`escape_confirmed=1`になること
- `stopping_reserve`が完了距離へ加算されていないこと
- P2のReverse/Drive gear要求回数が大幅に減ること
- `rolling_replan`が出て、primitive/steeringが現在姿勢から更新されること
- collision_worsening、wall pin、V2X blockerでは停止できること

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 targets）
- `colcon test-result --verbose`: 1046 tests、0 errors、0 failures、0 skipped
  - 別packageの古い`build/joycon_contract_guard/package.xml`欠損警告は出るが、
    今回対象packageのテスト結果とexit codeは正常。
- `make dev2`: 未実施。上記「動的確認」をユーザー試走で確認する。
