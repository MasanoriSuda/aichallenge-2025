# Task list

- [x] 最新走行の candidate rejection と ShiftOut speed cap を切り分ける
- [x] requirements と design を作成する
- [x] dynamic corridor admission policy を pure core に追加する
- [x] immutable `OvertakePassPlan` を pure core に追加する
- [x] candidate generator と mission freeze を plan/admission へ接続する
- [x] frozen ShiftOut footprint release を committed speed policy へ追加する
- [x] unit test を追加・更新する
- [x] 対象 package を build/test する
- [x] 動的効果確認項目を記録する

## 検証結果

- `make autoware-build`: 成功。25 packages finished。
- `ctest -R ^test_v2x_overtake_core$ --output-on-failure`: 成功。core 336 tests passed。
- `mpc_controller_cpp` target の再コンパイル: 成功。
- `make dev2` 実走: 未実施。AWSIM 上の競技シナリオ確認はユーザー側で実施する。

## 次回実走で確認するログ

1. `mission candidate selected` の `corridor_source=static_fallback` が、従来の `observed=0, samples=0, goal_candidates=0` 区間で出ること。
2. `OvertakeLine PassPlan frozen` が追い越し開始ごとに一度だけ出ること。
3. ShiftOut 中に `shiftout_footprint_release=1` となり、locked target 由来の adaptive cap が解除されること。
4. 動的重複を実際に観測したケースは `static_fallback` へ逃げず、従来どおり候補棄却または cap 再適用されること。
5. `Idle -> ShiftOut -> Pass -> Return -> Idle` の完遂数、`Pass -> Recovery` 数、前方 +8 m から後方 -8 m までの時間を前回 run と比較すること。

## 合格判定

- `observed=0` だけを理由とする candidate zero/cooldown が大幅に減る。
- footprint clear な ShiftOut で前車速度付近への長時間低下が発生しない。
- wall contact、current/predicted footprint overlap、target jump の件数を増やさない。
