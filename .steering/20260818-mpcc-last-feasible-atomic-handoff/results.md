# Results

## 実装結果

- 最新の solved MPCC trajectory と last physically validated trajectory の選択、
  現在進捗への整列、stage bounds / static-wall footprint 再検証を共通 helper へ集約した。
- target-bound failure でも、locked target continuity、現在車体非重複、短期予測sweep非重複、
  壁・front・solver hard guardを満たす場合は、直前のsolved軌道を実行prefixとして保持する。
- 不完全ShiftOutのsolved-prefix保持を0.75秒 / 4.0 mへ設定した。直前解が使えない場合は
  従来のcurrent-lateral freeze（0.35秒 / 2.0 m）へ戻る。
- solved-prefixの保持種別をMission状態へ記録し、新しいhorizonの0.20秒stable handoff中も
  0.75秒予算を維持する。handoff完了またはhard fault時に状態をatomicに解除する。
- 直前解の最大ageを0.75秒とし、各control callbackで進捗整列と物理再検証を行う。
- Pass/ShiftOutのMission generation、target、side、phase handoff互換を満たさない解は再利用しない。
- 予測欠損、予測sweep重複、通常body overlap、wall contact / margin / sample異常、
  EmergencyBrake、solver Recovery、target jump / course rejectionでは保持を解除する。

## 静的検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 28 / 28成功
- `test_v2x_overtake_core`: 708 / 708成功
- `test_mpcc_progress`: 19 / 19成功
- `colcon test-result --verbose`: 1249 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功

## 動的確認項目

比較元は `output/20260818-111623`。

- `target-bound execution hold started` の `mode=solved-prefix` が発生すること。
- ShiftOutの `hold ended without replacement` と直後のFollowPrepareが減ること。
- `prefix_source=last-feasible` の利用が0.75秒以内で、連続して長時間残らないこと。
- target jump、予測重複、壁接触、EmergencyBrake時にはsolved-prefixが即解除されること。
- ShiftOut -> Pass -> Return -> Idle完遂数が増え、接触・wall Recoveryが増えないこと。

動的効果確認は次回 `make dev2` 試走で行う。
