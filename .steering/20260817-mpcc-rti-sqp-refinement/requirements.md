# Requirements

## 目的

進捗・contour/lag 型 MPCC の追い越し実行層を、単一線形化点の QP から
RTI-SQP の最小構成へ進める。ShiftOut / Pass / Return 中に、直前の可行解を
使って非線形 Frenet 運動を再線形化し、同一制御周期内で解を1回改善する。

## 要件

- 1回目の QP 解を減衰更新した線形化点で、2回目の QP を解く。
- 経路曲率と操舵入力曲率を区別して線形化する。
- 2回目の再線形化または solve が失敗しても、1回目の可行解を制御に採用する。
- legacy MPC と Recovery の動作は変更しない。
- 既存の stage corridor、進捗 trust region、OSQP warm-start を維持する。
- 実行回数と減衰率は任意設定とし、未指定時は2回・0.65とする。
- 既存の `config.yaml` と `result-summary.json` のユーザー変更は変更・stageしない。

## Definition of Done

- RTI-SQPの減衰更新と入力検証に単体テストがある。
- 追い越しMPCC時だけ2回目の再線形化が実行される。
- refinement失敗時にfirst-feasibleへ戻る。
- package build/testが成功する。
- 今回の変更だけをコミットする。
