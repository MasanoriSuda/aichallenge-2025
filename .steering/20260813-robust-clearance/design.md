# Design

## 1. ロバスト余裕

設定値を次の形で解決する。

```text
vehicle_surface = clamp(base + speed_gain * speed
                              + curvature_gain * abs(curvature), 0, max)
vehicle_center_separation = max(legacy_center_separation,
                                physical_combined_width + vehicle_surface)

wall_reserve = clamp(base + speed_gain * speed
                           + curvature_gain * abs(curvature), 0, max)
wall_planning_clearance = hard_wall_clearance + wall_reserve
```

初期値では車両表面余裕を約0.25～0.30 m、壁計画余裕を約0.30～0.40 mとする。

## 2. 適用範囲

- 左右candidateのgoal intervalとtarget separation
- ShiftOut/Pass/Returnの静的preflightと実行horizon
- Pass front-capの解除・再適用
- body-clear handoff、tactical revalidation、forward escape延長
- safe trajectory prefixの現在車体／予測sweep分離
- runtime wall preplanのwarning footprint

## 3. hard guardとの分離

`v2x_overtake_line_min_wall_clearance`は物理hard guardとして0.20 mを維持する。
計画用余裕が成立しない場合はMissionを不成立または再計画対象にするが、既に接触した
ContactContinuationの分離動作に追加余裕を要求しない。
したがって通常Passはロバスト分離を満たすまで速度解放しないが、確認済みの
recoverable side contact中だけ実寸分離へフォールバックして前進復帰できる。

## 非対象

- localization filterの変更
- 車体寸法の変更
- ContactContinuation許可条件の変更
- 追い越し速度・加速度上限の変更
