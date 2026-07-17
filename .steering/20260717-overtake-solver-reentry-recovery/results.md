# 追い越しSolver復帰ゲート Results

実験日: 2026-07-18
比較run: `output/20260717-234612`
実験run: `output/20260718-000645`, `output/20260718-001009`

## 検証

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 43件成功
- `make dev3`: 同一binary・設定で2回実施
- `make down`: 両runとも成功

## Run 1: `output/20260718-000645`

D3がOvertakeLine非作動のCruise中にWP237付近で通常MPC failureへ入り、操舵`-0.436 rad`のまま停止した。
D2とD1は前方車をSafetyBrakeしてWP238 / WP236で停止した。対象のsolver re-entry gateが発火する前の
別事象であり、本実験の判定には使わない。

## Run 2: `output/20260718-001009`

### D2 gate sequence

1. WP86で`Idle -> ShiftOut`。
2. OSQP failureが8周期続き、`ShiftOut -> Recovery`。
3. solverは8連続failure後に復旧し、Recoveryを継続。
4. WP96のRecovery distance completeで、2秒cooldown＋20連続成功gateをarm。
5. 約2.03秒後、20 healthy solvesを確認してgateをrelease。
6. 再進入ループを作らず、D2はWP220まで走行。

D2はその後WP130でも追い越しを実行し、単発solver failureから次周期に復旧してPass / Recoveryを完了した。
元runのような停止後の`Idle -> ShiftOut -> Recovery`反復はなかった。

### 全車状態

| 車両 | 最終確認 | 判定 |
|---|---|---|
| D1 | WP321、5.44 m/s | 走行継続 |
| D2 | WP220、SafetyBrake | WP222で停止したD3を前方検出 |
| D3 | WP222、wall contact 61〜72 cells、SafeStop | 通常走行中の別事象 |

D1が走行を継続したため3台停止列ではない。D1/D3は前実験の対象WP72 / WP123を通過したが、D3は
後段WP222で通常走行のwall contactに入った。

## 未確認事項

Recovery中の連続failureが再現しなかったため、rate limit付き中立復帰の実commandは未観測である。
pure coreでは`-0.179 rad`が40 Hz・1.2 rad/sで1周期後`-0.149 rad`となり、絶対値が1周期最大
0.03 radずつ0へ近づくことをunit testで確認した。

## 判定

- solver-health re-entry gate: **Pass / 採用**
- Recovery fallback中立復帰: **unit Pass / 実走未発火**
- dev3全車継続走行: **Partial**（D3 WP222の別wall事象）
