# Design

## 方針

ContactContinuationが有効な間は、現行コードが既に以下を所有している。

- committed Pass Missionとsideの維持
- locked targetの回復可能な横接触に対するSafetyBrake抑制
- full-speed SafeSeparation forward escape
- 相手から離れる方向へのlateral bias
- horizon全体の壁境界によるbias clamp

したがって新しいFSMは追加せず、分類器へSchmitt-trigger型の相対横速度ヒステリシスを追加する。

```text
inactive:
  abs(relative lateral velocity) <= 0.5 m/s で開始可能

active:
  abs(relative lateral velocity) <= 0.8 m/s なら保持
  0.8 m/s超、または既存hard guard違反で解除
```

`previously_active`が変更できるのは横速度のrelease閾値だけであり、target、side geometry、時間、進捗、closing speed、ego speedの条件は毎周期再評価する。

## 動的確認

- `ContactContinuation entered/ended`の短周期反復回数
- `vlat_hysteresis=1`中のPass継続
- 回復可能な横接触直後のSafetyBrake回数
- `Pass -> Return -> Idle`完遂率
- 壁接触、wall pin、solver Recoveryの増減

