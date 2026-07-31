# 設計

## 事象1: static wall clamp後の横加速度超過

現行処理は、通常の横加速度limitを適用した後にstatic map clampを行う。clampでtargetが
変化すると横加速度を再計算するが、上限を超えた場合は再limitせず即Recoveryとしている。

次の順でbounded projectionを追加する。

1. 通常targetを横加速度上限内へ制限する。
2. wall margin付きfootprintでstatic map clampする。
3. clamp後targetが横加速度上限を超えた場合、到達可能な中間targetへ制限する。
4. 中間targetから現在offset方向を、marginなしの車体実寸footprintで再検査する。
5. 非接触なtargetを取得できた場合だけhorizonを継続する。

これは壁判定の無効化ではない。追加marginのみを一時的に緩和し、車体矩形の接触判定と
actual footprint guardは維持する。

## 事象2: 高速ShiftOut中の直接side反転

失敗直前は約6.06 m/s、`e_psi=-0.125 rad`、旧side=-1で、旧side方向の横速度は
約0.76 m/sだった。この状態でgoalを反対側へ切り替えた直後、OSQPが8周期連続で
maximum iterationsとなった。

次を計算する。

```text
locked_side_lateral_speed = max(0, locked_side * speed * sin(e_psi))
```

この値が設定閾値を超える間、`side_switch_permitted=false`とする。候補quality差だけなら
mission sideを保持し、実際のordering conflictなら既存Recoveryを使用する。

初期値は`0.25 m/s`とする。低速かつ横運動がほぼ止まった浅いShiftOutでは、既存の
early replanを引き続き利用できる。

## 診断

- `static_reachable=1`: acceleration-bounded physical targetを利用
- `replan_vlat`: 旧side方向の横速度
- `replan_vlat_ok`: 直接side switch許可状態

効果確認では、前回wp187付近で直接`side=-1 -> 1`とsolver連続失敗が発生しないこと、
wall clamp区間では接触なしでShiftOutが継続することを確認する。
