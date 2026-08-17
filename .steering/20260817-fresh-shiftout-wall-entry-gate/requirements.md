# Requirements

## Background

`f9825d1`後の実走`output/20260817-110115`では、2回の
`Pass -> Return -> Idle`が成立し、ReturnからRecoveryへ落ちる事象は解消した。

一方、episode 2ではStuck detectorが`wall=1`を観測した後に
`Idle -> ShiftOut`へ入り、直後のruntime wall warningが`ttc=0.00 s`となった。
安全なwall-escape prefixを生成できず、0.62秒後に
`actual footprint intersects static wall`でRecoveryへ移行した。

壁監視は新規Mission開始前にも評価されていたが、入口stageの認可に使われていなかった。

## Goal

- 現在車体が壁と接触中、または現在時点でrobust wall reserveを失っているとき、
  新規の横ShiftOutを開始しない。
- 将来予測だけのwall warningでは入口を止めず、既存のpreplan/prefix処理へ渡す。
- direct Pass、paused Mission、既に実行中のPassを中断しない。
- 壁状態が解消した後は、fresh candidateを通常どおり再評価できるようにする。

## Constraints

- 壁余裕、車体寸法、速度、横加速度、追い越し候補rankingのパラメータは変更しない。
- 入口保留時にcandidateをfreezeまたはside latchしない。
- active ShiftOut/Passのruntime wall処理は変更しない。
- ROS 2 topic/service、Domain、launch、result JSON schemaを変更しない。

## Definition of Done

- fresh lateral ShiftOut専用のwall entry gateをcore pure functionとして実装する。
- actual wall physical contactまたはcurrent wall warningでfresh ShiftOutを保留する。
- predicted warning、direct Pass、既存Missionには適用しない。
- core unit test、package test、buildが成功する。
