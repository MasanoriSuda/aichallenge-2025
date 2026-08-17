# Design

## 現状

現行MPCは空間領域モデルで、状態`[e_y, e_psi, t]`、入力`[v, kappa]`を使う。
固定距離ホライズンの所要時間`t`を小さくするため速度最適化能力はあるが、走行中の
実進捗そのものは状態ではなく、stageとwaypoint indexで外生的に固定されている。

## 第1段階の定式化

追い越し実行中だけ同じ3x2のQP構造を次へ切り替える。

```
x = [e_y, e_psi, s]
u = [v, kappa]
```

時間領域Frenet kinematic model:

```
s_dot     = v cos(e_psi) / (1 - kappa_ref e_y)
e_y_dot   = v sin(e_psi)
e_psi_dot = v kappa - kappa_ref s_dot
```

各stageは既存waypoint間距離と参照速度から得る有限な`dt`でEuler離散化し、
stage目標`e_y/e_psi`と参照進捗の周りで線形化する。DP corridorは従来どおり
`e_y`のhard boundへ入る。

目的関数の第3状態を次へ変更する。

```
J_s = q_lag (s - s_ref)^2 - q_progress s
```

`s`にはshifted/reference progress周辺のbounded trust regionを設定する。
後方側は速度capで進捗が遅れた場合にもfeasibleとなる幅を持たせ、前方側はhairpinの
別branchへ飛ばないよう狭くする。

## mode切替

- `Idle`、通常Cruise/Follow: legacy spatial-time MPC
- `ShiftOut`、`Pass`、`Return`: progress MPCC
- modeが変わった周期はOSQP historyをresetし、`t`と`s`を混同したwarm-startを防ぐ。

## 局所リファクタ

時間領域Frenet線形化、progress horizon、trust bound、cost係数を
`mpcc_progress` libraryへ分離する。巨大なcontroller内には、既存のMission/corridorを
QPへ接続するadapterだけを残す。

## Safety / compatibility

- 入力速度上限、曲率上限、操舵rate、stage corridorは既存hard constraintを維持する。
- denominator `1-kappa_ref e_y`が小さい線形化は拒否しlegacy solver fallbackへ渡す。
- ROS I/Oとlaunch構成は変更しない。
