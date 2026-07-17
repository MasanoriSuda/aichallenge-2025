# 壁接触予防・再合流安定化 実験結果

実施日: 2026-07-17
主run: `output/20260717-232948`
判定: Partial Pass / Follow-up Required

## 検証結果

| 項目 | 結果 | 根拠 |
|---|---|---|
| 前方危険targetの一時消失 | Pass | 危険観測後はhazard holdを維持し、直後のCruise復帰なし |
| pre-raceの誤hold | Pass | `WaitingForStart` / `Prepared`でholdをclearする修正後、主runでは発生なし |
| LowSpeedRejoin前進preflight | Pass | D1で前方wall collisionを検出し、再合流駆動を禁止 |
| 旧runの停止時刻約62秒超 | Pass | D2はStartから約62秒時点でWP 26を7.75 m/sで走行 |
| 最終的な3台停止回避 | Fail | D2のStartから約79秒後、D1/D3/D2の順で停止列が成立 |

## 主な時系列

- D3は先頭走行中、WP 72付近でlateral errorが増加し、現在footprintにwall contactを作った。
- D1はWP 123付近でコースを外れ、Recoveryは`ReverseRight`を選んだ。
- D1のLowSpeedRejoin前進preflightは約0.35 m先のwall collisionを検出し、駆動を禁止した。
- D3がD1後方へ到達したため、D1のreverse corridorが`rear_vehicle_blocked`となった。
- D1はclearance待機timeoutでSafeStopした。
- D3はD1を前方危険車両として検出し、約1.4〜1.6 mの間隔でSafetyBrakeを維持した。
- D2は旧停止時刻を超えて走行したが、後に停止列へ到達し、D3後方でSafetyBrakeした。

## 判定

今回の修正は、旧runで確認した「危険targetの短時間消失からCruiseへ戻って追突する」経路と、
LowSpeedRejoinが未確認の前方へ進んで新規wall contactを作る経路を閉じた。

最終的な全停止は別の閉塞である。先頭のD1には安全な前進rolloutがなく、後続D3が後退経路を塞ぎ、
D3とD2は安全側のSafetyBrakeを維持した。fail-closed条件を緩和して解消すべき状態ではない。

## 次段

1. WP 72 / WP 123付近で通常MPCがwall departureする原因を先に解析・抑制する。
2. 先頭車のwall departureを抑制しても停止列が残る場合、後続車が停止距離を確保する、または
   bounded reverseで退避空間を作るcooperative yieldを別ステアリングで設計する。
3. cooperative yieldではV2X completeness、rear-clear、static swept footprint、距離・時間・attempt上限を
   必須gateとし、単なるSafetyBrake解除や盲目的な後退は行わない。

## 補足run

`output/20260717-232628`では、静止グリッドを危険車両としてpre-race中にholdしてしまう副作用を確認した。
start-grid phaseが`WaitingForStart`または`Prepared`の間はholdをclearするよう修正し、主runで再発しないことを
確認した。
