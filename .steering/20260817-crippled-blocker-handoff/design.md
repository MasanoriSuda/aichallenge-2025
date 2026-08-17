# Design

## Observed failure

`output/20260817-195826/d1/autoware.log` では、停止車専用経路が壁 preflight で
反復棄却された一方、前方距離 5.11 m で通常 planner は壁余裕 1.98 m の完全
Mission を生成した。しかし `low_speed_candidate` が無条件に OvertakeLine を
停止車専用 planner へ譲らせ、Recovery 側も immediate Mission を handoff 対象に
含めなかったため、成立した経路を実行できなかった。

## Change

1. LowSpeedAvoidance/direct control が実際に有効な場合は従来どおり専用 planner を優先する。
2. 通常 Overtake が `validated_overtake_entry_immediate_execution` を持つ場合は、
   low-speed candidate が存在しても完全 Mission を OvertakeLine へ渡す。
3. 同じ即時 Mission を Recovery の forward Overtake handoff 対象へ追加し、
   検証済み complete Mission なので pre-arm 時間待ちを要求しない。
4. hard Recovery owner は変更しない。

## Expected behavior

停止車専用 local path が不成立でも、通常 planner が完全 Mission を見つけた周期に
`Idle -> ShiftOut` と Recovery handoff が成立する。完全 Mission がない場合は従来どおり
Follow / SafetyBrake / Recovery を維持する。
