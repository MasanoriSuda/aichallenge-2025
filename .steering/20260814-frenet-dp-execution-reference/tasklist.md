# Tasklist

- [x] 最新実走でDPが実行参照になっていないことを確認する
- [x] Mission、phase、receding-horizonの受け渡し箇所を確認する
- [x] DP execution referenceのpure coreを追加する
- [x] Mission candidate/stateへDP列を保持する
- [x] ShiftOut/Pass horizonのbaselineへDP列を接続する
- [x] runtime logを追加する
- [x] 単体テストを追加する
- [x] `make autoware-build`を実行する
- [x] package testを実行する（1118 tests、0 failure）

## 動的確認（ユーザー実施）

- [ ] `OvertakeLine DP execution: active=1`がShiftOut/Pass中に観測できる
- [ ] `physical target separation conflicts with wall bounds`が減る
- [ ] `Pass -> Return -> Idle`完遂率が上がる
- [ ] 壁接触とSafetyBrakeが増えていない
