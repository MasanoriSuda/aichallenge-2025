# Overtake Recovery追従引継ぎ実験 設計

## 現行の速度合成

縦速度上限は次の順でMPCへ適用される。

```text
V2X Behavior Follow / front risk
  ↓
gap planner
  ↓
OvertakeLine Recovery固定上限
  ↓
MPC
```

Recovery固定上限はBehaviorの後段にあるため、移動先行車との車間がすでに開いても
`v2x_overtake_recovery_velocity`が追加のhard capとして残る。

## 変更方針

Recovery中の速度上限を次のように選択する。

```text
solver Recovery、先行車ロスト・不連続・停止
  → 現行の固定Recovery上限

新鮮で連続した移動先行車あり
  → 通常Followと同じ距離連動上限
     距離ゲート内: 先行車速度 + 距離偏差フィードバック
     距離ゲート外: Recovery固有の追加上限なし

壁実接触
  → 0 m/s
```

通常Followの計算には既存`resolve_follow_speed_limit()`を再利用し、距離、速度閾値、
目標距離、上下速度マージン、距離gain、最大速度を重複実装しない。

固定上限とFollow引継ぎの選択は副作用のないcore関数へ切り出し、次を単体テストする。

- 有効な移動先行車がなければ固定上限
- 距離ゲート内は有限なFollow上限
- 距離ゲート外は追加上限なし
- solver Recoveryは常に固定上限
- NaNや負値は拒否

## ログ

既存の約1 Hz `OvertakeLine debug`へ`recovery_speed=fixed|follow`だけを追加する。
新しい高頻度ログは追加しない。

## 影響範囲

- ROSインターフェース変更なし
- yamlパラメータ追加・変更なし
- lateral trajectory、wall map、footprint、横加速度判定変更なし
- Follow単独時のロジック変更なし

## A/B評価

| 条件 | run | Recovery縦速度 |
|---|---|---|
| A | `20260724-235653` | 固定5.0 m/s |
| B | 今回取得 | 移動先行車のみFollow引継ぎ |

Autoware logとMCAPを時刻同期し、Recovery開始前後のcommand/actual speed、
実加速度、d1-d2位置と速度、次回ShiftOutまでの時間を比較する。

## 暫定採用時の既知課題

`20260725-005530`ではFollow引継ぎによってPassへ2回到達した一方、hairpin内側Pass後の
Recoveryで相手車両との進路が交差し、SafetyBrakeとStuck Recoveryへ進んだ。

暫定版では縦方向のFollow引継ぎを維持する。次の横方向課題はこの変更へ混ぜず、別の
ステアリングで扱う。

- hairpinで内側Passを選ばず外回りを優先する条件
- 相手車両の後方クリア確定まで復帰線へ戻さない条件
- 車両同士の接近・接触を静的壁contactと分離して記録するログ

競技シミュレーション向けの暫定方針であり、実車向け安全設計ではない。
