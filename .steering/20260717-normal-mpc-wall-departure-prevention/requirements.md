# 通常MPC壁逸脱予防 Requirements

作成日: 2026-07-17
状態: Experiment Complete（主目的Pass / dev3全体Partial）

## 目的

`output/20260717-232948`で、V2X車両との接触前に通常MPC走行だけで発生した
WP72およびWP121〜123付近のwall departureを予防する。

## Baseline evidence

- D3はWP67まで約2.75 m/sでCruiseし、WP72で`e_y=-1.964 m`、current wall contact 171 cellsとなった。
- 同時刻のMPC内部操舵は`-0.342 rad`で、publish時のgain 1.5によりAWSIM指令は約`-0.513 rad`となる。
- D1はWP120でFollowからCruiseへ戻った後、WP121で`e_y=-0.969 m`、`e_psi=-1.032 rad`となり
  OSQP failureへ入った。
- D1のMPC内部操舵`+0.463 rad`はgain適用後に約`+0.695 rad`となり、設定`delta_max_deg=32`の
  約`0.559 rad`を超える。
- 両事象とも後続車追突が原因ではなく、solver failureは大きなtracking errorの後に発生している。

## 仮説

`steering_tire_angle_gain_var=1.5`をMPC制約後に適用するため、MPCが予測する車両運動とAWSIM実運動が
一致せず、ヘアピンで過操舵となってtracking errorとsolver failureを誘発している。

## 実験要件

1. AWSIM向けgainを1.0へ変更し、モデル操舵とpublish操舵を一致させる。
2. trajectory、速度上限、Q/R、V2X、Recovery設定は同時に変更しない。
3. `make autoware-build`後に`make dev3`を実行する。
4. D3がWP72、D1がWP123をcurrent wall contactなしで通過するか確認する。
5. OSQP failure、`e_y`、`e_psi`、各車WP/速度、最終停止状態を比較する。

## 受け入れ条件

- D3がWP90以上へ到達するまでcurrent wall contactを作らない。
- D1がWP140以上へ到達するまでcurrent wall contactを作らない。
- WP72 / WP121〜123で連続OSQP failureを発生させない。
- 3台停止列を旧runのD2 Start後約79秒より前に作らない。

## 制約

- ROS topic/service/message、Domain、result JSONの契約を変更しない。
- RecoveryとV2Xのfail-closed gateを緩和しない。
- 本値は2025 AWSIM向けローカル実験値で、2026公式値ではない。

## 実験結果

`output/20260717-234612`で全受け入れ条件を満たした。D3はWP72を越えて1周、D1はWP123を
越えて1周し、両車ともStart後のcurrent wall contactおよび対象区間の連続OSQP failureはなかった。
旧runの全車停止時刻も越え、D1とD3は走行を継続した。

ただしD2は2周目のWP34〜41でOvertake ShiftOutからRecoveryへ遷移した後、別原因の連続OSQP
failureに入り停止した。このため通常MPCの壁逸脱予防はPass、dev3全体はPartialとする。
