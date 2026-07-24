# Overtake Recovery追従引継ぎ実験 結果

## 結論

**競技シミュレーション向けに暫定採用。ソース変更を維持する。**

移動先行車が有効なRecoveryで固定5.0 m/s上限を通常Followへ委譲する処理は作動し、
最大急減速と車間拡大が弱まり、現行では到達できなかったPassへ2回入った。

走行中止は、hairpinで内側Passを継続した後、相手車両との進路が交差して
SafetyBrake、solver失敗、Stuck Recovery、SafeStopへ進んだ事象と解釈する。
縦方向の改善は暫定採用し、hairpinのpass side選択と復帰条件を残課題とする。

これは課題出しを優先する競技シミュレーション用の暫定判断であり、実車適用を
意図しない。

## DATA

### A/B条件

| 条件 | run | Recovery縦速度 |
|---|---|---|
| A: 現行 | `20260724-235653` | 固定5.0 m/s |
| B: 実験 | `20260725-005530` | 移動先行車のみ通常Followへ委譲 |

d2は両方16 km/h、V2X/FSM/追い越し/安全設定と記録topicは同一。

### 周回・状態遷移

| 指標 | A: 現行 | B: 実験 |
|---|---:|---:|
| d1 Lap 1 | 75.193 s | 74.779 s |
| d1 Lap 2 | 74.948 s | 未完 |
| d2 Lap 1 / 2 | 73.694 / 75.023 s | 73.779 / 76.458 s |
| d1 Lap 1までのRecovery | 24 | 23 |
| d1 bag全体のRecovery | 57 | 24（停止後は再試行なし） |
| ShiftOut -> Pass | 0 | 2 |
| Stuck confirmed | 0 | 5 |
| Reverse intent | 0 | 1 |
| SafeStop | 0 | 1 |

Lap 1までのRecoveryは24回から23回であり、再試行頻度そのものは改善していない。
実験runでは周期debugのRecovery sample 17件中16件が`recovery_speed=follow`、
1件が`fixed`であり、変更ロジック自体は十分発火していた。

一方、現行では0回だったPassへ2回到達しており、引き離されて追い越し機会を失う
負のループは弱まった。

### Recovery直後のMCAP指標

| 指標 | A: 現行 | B: 実験 |
|---|---:|---:|
| 2秒内の最大実速度低下 | 3.623 m/s | 2.640 m/s |
| 2秒内の平均実速度低下 | 0.446 m/s | 0.554 m/s |
| Recovery窓の最小実加速度 | -11.449 m/s² | -3.674 m/s² |
| 2秒内の最大車間増加 | 6.131 m | 1.613 m |
| V2X平均rate | 16.332 Hz | 16.369 Hz |
| V2X最大gap | 0.073 s | 0.071 s |

最大急減速と最大車間増加は小さくなった。実験runはd1が1周後に停止しているため
全時間比較ではないが、縦方向の改善傾向とPass到達は確認できた。

### 失敗時系列

```text
1784908636.376  ShiftOut -> Pass
1784908640.255  Pass -> Recovery
                 reason=static wall clamp exceeds lateral acceleration limit
1784908640.579  recovery_speed=follow, v_limit=inf
                 locked targetは直後に後方扱い
1784908643.026  Follow -> SafetyBrake
                 front_distance=0.16 m
1784908643.510  OSQP失敗開始
                 7 consecutive failures
1784908649.677  Stuck confirmed
1784908650.028  Reverse intent latched
1784908651.202  SAFE_STOP
```

実接触sampleは`current_contacts=0`だったが、この値は静的壁側の判定であり、
相手車両との接触を否定できない。映像ではhairpin内側Pass後に相手車両と進路が
重なっており、ログでも`inner_pass=1`の直後にfront distance 0.16 mとなっている。

### MCAP健全性

- d1: 220.919 s、78,733 messages、5 topics
- d2: 231.597 s、82,549 messages、5 topics
- topic:
  - `/control/command/control_cmd`
  - `/clock`
  - `/localization/acceleration`
  - `/localization/kinematic_state`
  - `/v2x/vehicle_positions`

通信欠損やbag不足による判定ではない。

## OBSERVATION

通常Followへの委譲によってRecoveryの固定ペナルティが外れ、追い越し継続性は
改善した。Passへ2回到達したこと、最大車間拡大が6.131 mから1.613 mへ減ったことは
この方針を維持する根拠になる。

走行中止直前は`inner_pass=1`でhard curve内側を走り、PassからRecoveryへ移行した後に
相手車両が0.16 m前方として検出された。したがって、今回顕在化した主課題はRecovery
縦速度より、hairpin内側Passと復帰線が相手車両の走路へ重なることである。

ただしLap 1までのRecovery回数は24回から23回にしか減っておらず、同じ失敗条件への
即時再試行は別の残課題として残る。

## LIMITATION

- Bは安全停止のためd1を2周完遂させていない。
- 単発runであり、Pass成功率の統計評価には使えない。
- 2秒Recovery窓の指標は、その後に発生したSafetyBrake/Stuckの全減速を含まない。
- 車両同士の物理接触を直接示すcontact topicは今回の5-topic bagに含まれない。

そのため走行中止の直接原因は、映像とV2X/制御ログを合わせた推定である。

## NEXT

縦方向の暫定ロジックは維持し、次はhairpin対策を独立して扱う。

1. hairpinでは内側Passを抑止し、外回り候補を優先する。
2. 相手車両の後方クリアが確定するまで復帰線へ戻さない。
3. 車両同士の接近・接触ログを追加し、静的壁contactと区別する。
4. その後、失敗理由別cooldownと失敗sideの一時再選択禁止を検討する。

提出期限までの課題出しを優先し、このhairpin対策は残課題として後続作業へ回す。

## 実行した確認

- `git diff --check`: 成功
- 変更版`make autoware-build`: 25 packages成功
- 変更版coreテスト: 171件成功
- `make dev2`: d1 1周、d2 2周。d1 SafeStopで中止
- A/BのAutoware logと5-topic MCAP解析
- 実験による`result-summary.json`差分を復元
- ユーザー判断によりFollow引継ぎロジックを暫定版として再適用
- 再適用版`make autoware-build`: 成功
