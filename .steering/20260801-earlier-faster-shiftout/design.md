# 設計

Candidate Aの距離変更を取り下げ、Candidate Bでは相対速度だけを変更する。

| 設定 | Baseline | Candidate A | Candidate B |
|---|---:|---:|---:|
| `v2x_follow_distance` | 5.0 m | 8.0 m | 5.0 m |
| `v2x_overtake_guard_min_prepare_distance` | 3.0 m | 6.0 m | 3.0 m |
| `v2x_overtake_guard_min_front_distance` | 3.0 m | 6.0 m | 3.0 m |
| `v2x_overtake_shiftout_min/max_closing_speed` | 0.5/1.2 m/s | 0.8/2.0 m/s | 0.8/2.0 m/s |

`v2x_follow_speed_limit_distance`と`v2x_moving_follow_target_distance`は3.0 mのまま
維持する。Candidate Bはentry可否をBaselineへ戻し、安全距離の予算がある間の
adaptive closing speedだけを比較する。

6～8 mでの早期prepositionが必要な場合は、entry禁止距離を流用せず、準備開始条件を
独立させる。Candidate Bではそのロジック変更を行わない。
