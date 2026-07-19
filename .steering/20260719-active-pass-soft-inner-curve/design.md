# Design

## ポリシー

`v2x_overtake_continue_inner_soft_curve_enabled` を追加する。既定は `false` とし、dev3用の
現行configだけ `true` にする。

pure coreにcurve continuation policyを置き、次の全条件を満たすときだけsoft-inner継続を許可する。

- FSMがすでにOvertake
- `v2x_overtake_continue_in_forbidden_enabled=true`
- soft curvature forbidden
- cooldownなし
- EmergencyBrakeなし
- 明示禁止WPなし
- `v2x_overtake_completion_hard_curvature`以上のhard curveがMPC horizon内にない

controllerでは通常front passとside passの両方で同じpolicyを使用する。新規開始側の
inner-curve blockは変更しない。

## 互換性

ROS 2 topic/service/messageは変更しない。新規yaml keyを省略した場合は`false`で従来挙動を維持する。
