# Design

## Existing assets

左右の`OvertakeMissionCandidate`には、同一のrolloutと設定で算出された以下がすでに保存されている。

- `horizon_progress_score`
- `predicted_rear_clear_time_sec`
- `predicted_minimum_ego_speed_mps`
- path/corridor/Return clearance

これらを再計算せず、`PassManeuverCandidateAssessment`へ射影して比較する。

## Ranking policy

現在側が不成立で反対側が成立する場合は反対側を選ぶ。両側成立時は次のいずれかを満たす候補を切替候補とする。

1. rear-clear時間が設定値以上短く、reserveと最低速度の後退が許容内
2. horizon progress scoreが設定値以上高く、reserveの後退が許容内
3. 物理reserveが従来閾値以上高く、rear-clear時間と最低速度の後退が許容内

無限値や未評価値は明示的に扱う。rankingを無効にした場合は従来のreserve比較へ戻す。

## Initial parameters

- rear-clear time advantage: 1.0 s
- progress score advantage: 0.35
- maximum reserve regression: 0.05 m
- maximum rear-clear time regression: 0.25 s
- maximum minimum-speed regression: 0.25 m/s

評価周期0.15秒、stable 0.25秒、no-return 3.5 m、最大置換1回は現行を維持する。

## State ownership

`compare_opponent_side_maneuvers()`は純粋なランキングのみを所有する。以下は従来どおり`resolve_opponent_side_replan()`が所有する。

- target continuity
- body/predicted footprint gate
- no-return
- replacement count
- debounce完了後のatomic Mission replacement

## Observability

周期ログとpendingログへ以下を追加する。

- ranking reason
- rear-clear time advantage
- horizon progress score advantage
- minimum-speed advantage
- physical reserve advantage

