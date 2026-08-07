# Tasklist

- [x] `20260808-063457`のdynamic wait結果を集計する
- [x] 要件と設計を作成する
- [x] paused Missionのlocked target幾何観測を追加する
- [x] dynamic wait入場条件をpure policy化する
- [x] no-return診断を修正する
- [x] 単体テストを追加する
- [x] 永続仕様を更新する
- [x] build/testを実行する
- [x] 実走確認項目を記録する

## 実走確認

`make dev2`で次を確認する。

- `mission_wait=1`中に`opp_eval=1`が観測される。
- locked targetが縦方向に離れた場合、`body_clear=1`へ更新される。
- dynamic waitがno-return後に新規発生しない。
- `dynamic mission wait released`または`opponent side PassPlan replaced`が発生する。
- FollowPrepare time limitが8回から減少する。
- Pass -> Return -> Idle完遂率が`20260808-063457`の1/12を上回る。

## 静的検証結果

- `make autoware-build`: 成功（25 packages）。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test targets）。
- `V2XOvertakeCoreDynamicMissionWait.*`: 4 tests、失敗0。
- `git diff --check`: 成功。
- `make dev2`: 未実施。上記の実走確認項目は次回走行ログで判定する。
