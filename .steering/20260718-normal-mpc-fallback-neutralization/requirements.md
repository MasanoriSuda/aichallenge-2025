# 通常MPC Failure操舵中立復帰 Requirements

作成日: 2026-07-18
状態: Experiment Complete（中立復帰Pass / dev3停止解消Fail）

## 背景

`output/20260718-000645`のD3は、Cruise中のWP237付近でOSQP failureへ入り、最初のfailure時点で
`speed=1.292 m/s`、`steering=-0.436 rad`、`e_y=1.315 m`、`e_psi=1.385 rad`だった。その後
407周期、約10.17秒にわたり、減速速度が0になった後も同じ操舵を保持した。solverが一度復旧すると
大きな横・方位誤差のまま再発進し、再failureを繰り返して壁際へ到達した。

`output/20260718-001009`のD3 WP222停止は、solver failureなしで`e_y=-1.650 m`とwall contactへ
入った別事象である。本ステアリングの主判定には含めず、dev3回帰観測として記録する。

## 要求

1. 通常MPCの単発solver failureでは、直前操舵を短時間保持して不要な操舵跳びを避ける。
2. 設定周期を超えてfailureが連続した場合、既存の減速fallbackを維持しながら操舵を中立へ戻す。
3. 中立復帰は`steer_rate_max`と制御周期を超えない。
4. OvertakeLine Recoveryのsolver failureは、従来どおり待機周期なしで中立復帰する。
5. 待機周期はYAMLで設定でき、負値を拒否する。
6. failureログから操舵がholdかneutralizeか判別できる。
7. ROS topic、service、message、Domain、評価JSON契約を変更しない。

## 暫定値

- `solver_failure_steering_hold_cycles: 4`
- 40 Hzでは0.1秒。5周期目から最大`1.2 rad/s`で中立へ戻す。
- 2025 AWSIM向けの実験値であり、2026公式値ではない。

## Definition of Done

- pure coreで境界、強制中立、異常値をunit testする。
- `make autoware-build`が成功する。
- `test_v2x_overtake_core`が成功する。
- `make dev3`を実行し、D1〜D3のsolver failure、操舵、wall contact、最終進捗を解析する。
- 対象の長時間failureが発火した場合、設定周期後に操舵絶対値が中立方向へ減少する。
- 対象failureが再現しない場合はlive判定をInconclusiveとし、unit/build結果と回帰観測を分ける。

## 完了判定

`output/20260718-003725`のD3で対象failureが再現し、4周期hold後の5周期目から操舵が
`0.559 -> 0.529 -> 0.379 -> 0.079 -> 0.000 rad`と中立方向へ減少した。要求1〜7は満たしたため
中立復帰機能は採用する。一方、D3は約`-1.8 rad`の大きな方位誤差からsolverが復旧せず、
Stuck Recoveryも`forward_duration_limit`でSafeStopした。D1/D2も前方のD3にSafetyBrakeし、
全車停止の解消はFailである。詳細は`results.md`に記録する。
