# Design

## 観測結果

20260813の2走では11回の追い越し開始がすべてPassへ到達した一方、正常なReturn完遂は1回、Recovery遷移は6回だった。特にstatic fallbackで横移動1.94 m、横加速度5.22 m/s^2、body-clear余裕0.36秒の候補が実行され、その後に壁・short horizon問題へ入った。

## 方針

Progressive Entryを完全Missionへ戻さず、入口と直後の脱出区間だけを検証する。

1. 従来どおりbody-clear rolloutを実行する。
2. deadline slackが0.6秒未満ならProgressive Entry候補から除外する。
3. `predicted body-clear distance + 6.0 m`まで同じ横目標を維持するShiftOut/Pass経路を、静的wall bounds・車体footprint・横加速度でpreflightする。
4. 成立した候補だけをProgressive Entry候補集合へ入れる。
5. 実際に検証できたPass距離を`static_valid_until_pass_m`へ保存し、未検証の12 mを検証済みとして扱わない。
6. Complete Missionが存在する場合は従来どおりCompleteを優先する。

entry準備候補と実行可能Progressive候補を別vectorにする。これにより短期ゲート不成立でも、前方車へ追いつくための既存speed preparationは維持される。

## 設定

- `v2x_overtake_progressive_entry_min_body_clear_slack_sec: 0.6`
- `v2x_overtake_progressive_entry_short_continuation_distance: 6.0`
- `v2x_overtake_progressive_entry_static_fallback_max_lateral_shift: 1.8`
- `v2x_overtake_line_min_wall_clearance: 0.20`

ローカル設定と提出用cloud設定は同値にする。

壁余裕は車体footprint外側へ適用される。従来値から片側5 cmのみ増やし、狭い区間の候補を一気に失わない範囲で実wall接触を抑える。runtime wall preplan reserve 0.10 mは維持するため、事前警告帯は実質0.30 mとなる。

## ログ

候補探索ログへ次を追加する。

- deadline slackによるProgressive候補棄却数
- short continuation preflightによる棄却数
- 選択時の設定slackとcontinuation距離

周期ごとの追加ログは行わない。
