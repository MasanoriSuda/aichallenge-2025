# Design

## 現象

初回 Mission は予測 rear-clear までの Pass 距離で採用される一方、実行時 SafeSeparation は追加の完遂距離 margin を要求する。そのため採用後に静的 Pass horizon が不足し、replan/SafeSeparation へ移る。また course-role の左右比較が短い reserve のみを見ると、実行時に延びた rear-clear が曲率反転を越えてから outer-to-inner を検出する。

## 方針

1. `resolve_overtake_dynamic_pass_distance` に runtime completion reserve を追加する。
   - SafeSeparation が使う `completion_distance_margin` と同じ値を初回 Pass hold に含める。
   - hard Pass distance limit は従来通り守る。
2. runtime course-role reserve を pure function で決める。
   - `max(configured_role_reserve, revalidation_lead_distance + completion_distance_margin)`
   - 現行設定では `max(2.0, 3.0 + 1.0) = 4.0 m`。
3. 左右 Mission candidate の rear-clear role をこの reserve まで評価する。
   - outer-to-inner が見える候補は既存の global Mission comparator で不利になる。
   - inner-to-outer は引き続き有効候補とする。
4. ログへ実際に使った runtime reserve を出す。

## 安全・互換性

- ROS topic/service/message 契約は変更しない。
- 絶対時間・距離上限、壁、車体、横加速度 guard は維持する。
- 余裕を含めて hard distance 上限を超える候補は admission で不成立となる。

