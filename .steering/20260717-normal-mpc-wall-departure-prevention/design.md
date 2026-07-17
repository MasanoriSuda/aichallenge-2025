# 通常MPC壁逸脱予防 Design

作成日: 2026-07-17
状態: Experiment Complete（主目的Pass / dev3全体Partial）

## Phase 1: steering model/output alignment

`mpc.steering_tire_angle_gain_var`を1.5から1.0へ変更する。MPCの`delta_max`、horizon先頭の
steer rate、bicycle modelへ渡す操舵と、`/control/command/control_cmd`へpublishする操舵を一致させる。

この実験ではwall位置固定の速度制限や強制停止を追加しない。まず通常制御のモデル不一致を除去し、
WP72 / WP123の再現性を確認する。

## 判定

- Pass: 対象WPをcontact/solver burstなしで通過する。
- Partial: 一方だけ改善する。残る区間のtrajectory/制約を次phaseで解析する。
- Fail: 逸脱が同等または悪化する。gainを戻し、normal forward swept-footprint guardを設計する。

## Rollback

`steering_tire_angle_gain_var: 1.5`へ戻す。その他の設定は変更しない。

## 実験判定

Phase 1を採用する。`output/20260717-234612`ではD3のWP72、D1のWP123をcontact / solver burst
なしで通過し、両車が1周を完了した。gain 1.0によりMPCモデル内操舵とAWSIM publish操舵を一致
させる方針は、今回の2025 AWSIM環境で有効だった。

D2の2周目WP34〜41の停止は、OvertakeLineのShiftOut中に対象を失ってRecoveryへ入り、横誤差が
拡大した状態でsolver failureと追い越し再進入が継続した事象である。壁接触はなく本設計とは原因が
異なるため、gain変更へ混ぜず別ステアリングでre-entry guardとsolver fallbackを設計する。
