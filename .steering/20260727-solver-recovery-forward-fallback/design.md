# Design

## 原因

solver fallbackかつheading errorが閾値以上の場合、
`recovery_reverse_only_episode_`がラッチされる。これは最初の退避方向を
Reverseへ限定するために必要だが、現在実装ではSafeStop後も解除されない。

今回のP1ではReverseのStraight/Left/Rightがすべて静的collisionとなる一方、
Forward rejoinはclearだった。Reverse-onlyによりForward primitiveを評価せず、
`maneuver_direction_unknown`となった。

## 解禁ゲート

SafeStopからaggressive retryへ遷移した時点で、次をすべて満たす場合だけ
Forward fallbackを解禁する。

- simulation環境かつaggressive simulation recovery有効
- solver fallbackが継続中
- solver由来のReverse-only episode
- wall分類がNone
- 現在footprintがclear
- Reverse候補を1姿勢以上評価済み
- Reverse候補が選択されず、診断理由が静的collision
- forward rejoin rolloutがclear
- V2X message/corridorがcompleteかつclear
- Boost inactive確認済み

解禁時はReverse episode/intentラッチを解除し、
`recovery_forward_fallback_unlocked_`を保持する。

## 候補選択

wall=Noneかつ現在footprint clearの場合、従来通りReverse候補を先に評価する。
解禁後にReverse候補が存在しなければForward Straight/Left/Rightを評価する。
Reverse候補が再び成立した場合も、既存のalternate-forward処理により
Forward rolloutを比較できる。

選ばれたForward候補は既存処理で次を再確認する。

- swept static footprint
- V2X moving/stationary corridor
- gear report/shift
- forward速度・距離・時間上限
- contact worsening

## 互換性

- ROS interface: 変更なし
- パラメータ: 追加なし
- 評価成果物: 変更なし
- 通常走行・OvertakeLine: 変更なし
- simulation-only制約: 維持
