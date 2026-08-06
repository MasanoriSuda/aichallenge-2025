# Design

## ShiftOut predicted-overlap debounce

既存の `v2x_overtake_pass_predicted_overlap_confirm_sec` と確認clockを、次の条件を満たすShiftOutでも共有する。

- frozen full missionがある
- body-clear deadlineが検証済みで成立している
- minimum-motion corridorを実行中
- front capを既に解除済み
- locked targetのcurrent body footprintは分離
- target continuityとfootprint predictionが有効
- predicted sweepだけがoverlap

確認時間未満は解除状態を保持する。継続確認後は既存どおりcapを再適用する。初回のcap解除には使わない。

## Rear-clear observation retention

target観測が一時的に消えた場合でも、target hold時間内かつ最後のcourse longitudinalが
`-return_clear_distance` 以下ならrear-clear観測を短時間保持する。

`clear_confirm_sec` を満たした後は、generic side classificationが同一targetをside vehicleとして残していても、return corridorが非blockならReturnを許可する。position jumpとsolver recoveryは常に優先する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: pure policyの入力・判定
- `mpc_controller_cpp.cpp`: confirmation clockとlast observationの接続
- `test_v2x_overtake_core.cpp`: ShiftOut debounce、rear-clear target lossの回帰試験
- 設定値およびROS interfaceは変更しない
