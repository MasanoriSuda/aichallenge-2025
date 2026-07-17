# 追い越しSolver復帰ゲート Requirements

作成日: 2026-07-17
状態: Experiment Complete（gate Pass / dev3全体Partial）

## 目的

`output/20260717-234612`のD2で、2周目WP34〜41のOvertakeLine Recovery中にMPC solver failureが
連続し、停止後も追い越しへ再進入してfailureを継続した事象を解消する。

## Baseline evidence

- D2はWP34で`Idle -> ShiftOut`へ入り、WP37でtarget消失により`ShiftOut -> Recovery`へ遷移した。
- Recovery開始後、操舵`-0.179 rad`を固定したdeceleration fallbackが継続し、横誤差は
  `-0.39 -> -2.01 m`へ拡大した。
- WP41で停止後も、Recovery stall解除から約0.6秒で追い越しへ再進入した。
- solver abort後には2秒cooldownがarmされたが、solver成功を一度も確認しないまま期限だけで解除され、
  `Idle -> ShiftOut`を繰り返した。
- Start後のmap wall contactはなく、前実験の通常MPC操舵gain問題とは別事象である。

## 実験要件

1. OvertakeLine Recovery中のsolver fallbackでは、操舵をrate limit付きで中立へ戻しながら減速する。
2. Recovery中にsolver fallbackを検出したepisodeは、終了時に追い越し再進入gateをarmする。
3. gateは既存cooldown時間に加え、通常MPC解の連続成功を確認するまで解除しない。
4. gate中にsolver failureが再発した場合、成功カウントを0へ戻す。
5. SafetyBrake、LowSpeedAvoidance、wall recovery、V2X fail-closed条件は緩和しない。
6. unit test、`make autoware-build`、`make dev3`の順で検証する。

## 受け入れ条件

- D2が2周目WP41で連続solver failureのまま追い越し再進入を繰り返さない。
- solver未復旧中は`Idle -> ShiftOut`へ遷移しない。
- D2がWP60以上へ再発進するか、少なくとも安全停止を保持してD1/D3を巻き込まない。
- D1/D3の前実験で解消したWP72 / WP123壁逸脱を再発させない。
- Start後の新規wall contactおよび3台停止列を作らない。

## 制約

- ROS topic/service/message、Domain、result JSON契約を変更しない。
- trajectory、操舵gain、速度上限、Q/Rは同時に変更しない。
- 値は2025 AWSIM向け暫定実験値で、2026公式値および実車値ではない。

## 実験結果

`output/20260718-001009`のD2で、WP86のOvertake ShiftOut中に8周期のsolver failureが発生し、
Recoveryへ遷移した。solverは直後に復旧し、Recovery完了時にgateをarmした。既存2秒cooldownと
20回の連続成功を満たした後だけgateを解除し、D2は再進入ループなしでWP220まで走行した。

元runのWP41連続failure自体は再現しなかったため、中立復帰fallbackの実走発火は未確認で、pure core
unit testでrate limitを確認した。D3が別の通常走行WP222でwall contactへ入り、D2がSafetyBrakeした
ためdev3全体はPartialとする。D1はWP321・5.44 m/sまで走行し、3台停止列にはなっていない。
