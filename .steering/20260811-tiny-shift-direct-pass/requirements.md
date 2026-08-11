# Requirements

## 目的

既にpass corridor内にあり、必要横移動が微小な検証済みMissionをShiftOutへ入れず、
Passとして開始して前進・追い越し所有権を維持する。

## 観測事実

`output/20260811-173943/d1/autoware.log`では次のMissionが失敗した。

- entry: `current_ey=0.02 m`, `goal_ey=0.10 m`, `lateral_shift=0.08 m`
- `body_clear_t=0`, `body_clear_s=0`, full Mission preflight済み
- entry直後にbody footprintとprediction sweepはclear
- それでも`Idle -> ShiftOut`となり、約0.47秒後にBehaviorがFollowへ戻った
- その後SafetyBrake、paused target loss、Recovery、Reverseへ連鎖した

## 必須要件

1. 現在位置が採用corridor内で、全Mission検証済み、必要横移動が設定値以下なら
   DirectPassとして分類する。
2. 既存のbase racing line DirectPassは維持する。
3. 閾値を超える横移動は従来どおりShiftOutとする。
4. wall/corridor、body-clear、rear-clear、Returnの既存preflightを省略しない。
5. start-grid、pause resume、active Mission replacementの挙動は変更しない。
6. dev/cloud設定は同じ値にする。
7. ROS 2 interface、launch、topic、message型を変更しない。
8. `aichallenge/result-summary.json`を編集・コミットしない。

## 初期値

```yaml
v2x_overtake_minimum_motion_direct_pass_max_lateral_shift: 0.20
```

## 完了条件

- pure core testでbase-line、tiny-shift、閾値超過、corridor外を固定する。
- entry stage testでtiny-shiftの優先順位とreasonを固定する。
- 対象packageのtest/buildが成功する。
