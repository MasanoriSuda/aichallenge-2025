# Design

## Preview / Action 分離

`RuntimeWallPreplanRequest`へ`action_required`を追加する。既定値は`true`とし、既存呼び出しのfail-safe動作を維持する。

- prediction warningかつTTCが`runtime_wall_preplan_action_ttc_sec`より大きい: preview
- 現在位置のwarning、またはTTCが閾値以下: action
- hard wall fault: 従来どおりruntime preplanでは上書きせず、既存hard guardへ委譲

previewでもfresh same-side Missionとcenter contractionは採用できる。ただし候補が得られない場合は`RequestFreshSameSideCandidate`または`HoldCurrentSide`とし、`ExitCurrentMission`へ移行しない。

## Escape prefix horizon

`RuntimeWallEscapePrefixHorizonRequest`へ`maximum_shift_distance_m`を追加する。prediction warning時は、

`available = speed * TTC`

からhold距離を除いた残りをshiftへ使い、`configured_shift_distance_m`以上かつ`maximum_shift_distance_m`以下まで延長する。予告距離が短い場合は従来どおりshiftを短縮する。

これにより、例えば速度約5.6 m/s・TTC 1.2 sなら、従来の4 m固定ではなく約5.7 mのshiftを使え、同じ横移動量の必要横加速度を下げられる。

## 設定

- `v2x_overtake_runtime_wall_preplan_action_ttc_sec: 0.50`

lookahead 1.20 sは探索開始に使い、0.50 sを中断判断の初期値とする。

## 検証

- previewでcontraction不成立でもMissionをexitしない単体テスト
- action帯では従来どおりexitできる単体テスト
- 利用可能距離に応じてshiftが4 mを超えて延長され、max shiftを超えない単体テスト
- YAML整合、package build、既存core test
