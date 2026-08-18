# Tasklist

- [x] 最新走行の追従回帰を切り分ける
- [x] stage/terminalの横・姿勢追従重みを分離する
- [x] hard bound内のmode handoffを実装する
- [x] mode handoff単体試験を追加する
- [x] `make autoware-build` を実行する
- [x] package testを実行する
- [ ] 動的試走で `Pass -> Return`、横誤差、姿勢誤差を確認する

## Definition of Done

- build/testが成功する
- front-risk速度上限をhandoffが超過しない
- 拡張MPCCの数値安定化機能を残したまま追従重みを回復する
