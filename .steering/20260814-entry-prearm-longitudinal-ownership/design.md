# Design

## 1. Pre-arm longitudinal reference floor

pre-armは速度上限ではなく、追い越し入口に必要なclosing speedを作るための縦方向参照として
扱う。各horizon点で次を適用する。

```text
reference = min(dynamic_hard_cap, max(base_reference, prearm_target_speed))
```

後段のfront-risk emergency、SafetyBrake、曲率・車両制約などのhard limitは従来どおり優先する。

## 2. Same-target validation lease

current cycleでcandidate生成に失敗しても、直前まで同一targetのpre-armが成立し、validation
lease・pre-arm window・hard guard・最低前方距離が成立する間は縦方向pre-armのみ継続する。

leaseはlateral Missionを復元しない。freshな完全Missionが再成立し、相対速度ゲートを満たした
周期だけOvertakeLineへ横制御をhandoffする。

## 3. 非対象

Pass中のSafeSeparation設計、MPCC-lite非同期化、Recovery、壁余裕の攻撃化は変更しない。
