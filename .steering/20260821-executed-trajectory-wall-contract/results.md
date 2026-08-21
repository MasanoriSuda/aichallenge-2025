# Results

## 実装結果

参照経路の物理証明と、solverが実際に出力する状態列の物理証明を分離した。
ShiftOut / Pass / Returnでは、solver成功後かつcontrol warm start更新前に実状態列を
車体掃引で検証する。

不成立時はsolver failureへ変換せず、次のfailoverを選ぶ。

- 未走行ShiftOut: `entry-rollback`
- 走行済みShiftOut / Pass: `dynamic-replan`
- Return: `recovery-replan`

拒否された解はcontrol、prediction、last-feasible trajectoryへ採用しない。出力は直前の
安全な操舵を保持し、速度は1制御周期分だけ減速する。Boostは抑止する。

## 静的検証

- `make autoware-build`: 成功
- package full test: 32/32 targets成功
- test result: 1451 tests、0 errors、0 failures、0 skipped

## 動的検証

未実施。次回`make dev2`では、旧run `output/20260821-222837/d1`で発生した
entry acceptedから38 ms後のfinal wall rejectionが、発行前の
`Overtake executed solution wall contract`で遮断されることを確認する。
