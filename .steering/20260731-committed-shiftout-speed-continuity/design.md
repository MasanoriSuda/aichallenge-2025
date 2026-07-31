# 設計

## 原因

現行の 3.0 m/s reference floor は、Pass、lateral completion、front-cap release、
overlap latch がすべて成立した後だけ有効になる。

一方、通常 Overtake の ShiftOut -> Pass 遷移は走行距離と line goal 到達も要求する。
相手位置や corridor goal が動くと、実際には相手の横へ出ていても ShiftOut が長引き、
停止対象速度 + closing-speed cap が参照速度を所有し続ける。

## 変更

`CommittedPassSpeedFloorRequest` に ShiftOut と現在横離隔の入力を追加する。

- Pass の既存条件はそのまま維持する。
- ShiftOut では phase 完了や Pass latch の代わりに、現在の完全な物理横離隔を要求する。
- 共通条件として current observation、physical path feasibility、非接触、低速 target を要求する。

controller にはすでに course-frame で算出した
`locked_target_current_lateral_clear` があるため、新しい幾何計算や閾値は追加しない。

## 速度権限

この変更は `ur` の reference を既存 3.0 m/s まで持ち上げるだけであり、
front-cap state は release しない。

```text
target closing-speed reference
        ↓ max(reference, 3.0 m/s)
curvature / wall / SafetyBrake / MPC hard bounds
        ↓ min(hard bound)
final reference
```

したがって、横離隔が成立した停止車の横での crawl は抑えるが、壁や別の前方車へ
全開で進む変更にはしない。

## 診断

周期 debug に次を追加する。

- `shift_floor=1`: ShiftOut 条件で reference floor が有効
- `floor_active=1`: Pass または ShiftOut の floor が有効

実走では ShiftOut 中に `shift_floor=1`, `v_floor=3.00` となり、
SafetyBrake 等がない区間で参照速度が停止対象速度まで落ちないことを確認する。
