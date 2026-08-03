# Requirements

## Background

`output/20260804-001441`では、D1が18回Passへ入ったものの、`Pass -> Return`は0回、
`Pass -> Recovery`は18回だった。主な失敗は次のとおり。

- `atomic commit: prediction expired`: 11回
- outer passがrear-clear前にinsideへ反転: 3回
- full mission pathの横加速度超過: 2回
- fresh target prediction unavailable: 2回

`prediction expired`には、`ShiftOut -> Pass`から約0.9 ms後に発生した例がある。
コード上、予測期限はROS時刻由来だが、延長計画のcommit時刻は`steady_clock`由来で、
異なる時計系を直接比較している。

SafeSeparationは前車が2 m以上前方にいる1周期だけで`RecoverBehind`を返すため、設定された
3秒/8 mの範囲を使わず、約25 msでRecoveryへ落ちている。

## Goals

1. same-side Pass延長計画の生成時刻、commit時刻、予測期限を同じROS時刻基準で比較する。
2. 前方クリアを連続確認してから`RecoverBehind`へ移り、単周期の観測でPassを中断しない。
3. Emergency、short-horizon unsafe、timeout、distance上限、rear-clear Returnの既存優先順位を維持する。
4. ROS topic/service、launch、評価結果schema、グローバル`a_max=1.0`を変更しない。

## Non-goals

- outer/inside反転判定や横加速度上限の緩和
- horizon progress scoreの再調整
- Stage 2の速度ホライズンownership
- Recovery/Stuck FSMの変更

## Acceptance criteria

- 計画処理時間をROS時計へ写像する単体テストまたは同等の回帰テストが成功する。
- SafeSeparationは前方クリア直後にはsame-sideを維持し、設定時間の連続確認後だけ
  `RecoverBehind`を返す。
- short-horizon unsafe、timeout、rear-clear Returnの既存テストが成功する。
- `test_v2x_overtake_core`と`make autoware-build`が成功する。
- 次回`make dev2`で`atomic commit: prediction expired`とPass直後のRecovery回数を確認できる。
