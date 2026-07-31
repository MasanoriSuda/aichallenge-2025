# 設計

変更対象は次の4設定だけとする。

| 設定 | Baseline | Candidate | 意図 |
|---|---:|---:|---|
| `v2x_follow_distance` | 5.0 m | 8.0 m | 早い段階から前方車をFollow準備対象にする |
| `v2x_overtake_guard_min_prepare_distance` | 3.0 m | 6.0 m | 短すぎる空き区間への新規ShiftOutを避ける |
| `v2x_overtake_guard_min_front_distance` | 3.0 m | 6.0 m | 近づき過ぎた状態からの新規ShiftOutを避ける |
| `v2x_overtake_shiftout_min/max_closing_speed` | 0.5/1.2 m/s | 0.8/2.0 m/s | 距離余裕がある間に早く並走位置へ進む |

`v2x_follow_speed_limit_distance`と`v2x_moving_follow_target_distance`は3.0 mのまま
維持する。8 mから先行車速度へ合わせる変更ではなく、8 mから候補経路を準備し、
安全距離の予算がある間だけadaptive closing speedを増やす実験とする。

新規entryに6 mを要求するため、スタート直後など初回認識時点ですでに6 m未満のケースは
ShiftOutしない。試行回数が大きく減る場合は本Candidateを不採用とし、entry判定と
preposition開始判定を分離する次段階へ進む。

