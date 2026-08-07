# Tasklist

- [x] 現行ログと既存shadow replanの到達条件を照合する
- [x] 要件・設計を作成する
- [x] FollowPrepareを左右Mission再評価対象へ追加する
- [x] dynamic mission wait状態とsoft-failure遷移を追加する
- [x] paused Missionのsame-side再開／alternate置換を接続する
- [x] ログと永続仕様を更新する
- [x] pure core / controller単体テストを追加する
- [x] build/testを実行する
- [x] 実走確認項目と結果を記録する

## Definition of Done

- 現側不成立・反対側未成立で、hard faultがなければRecoveryではなくFollowPrepareを保持する。
- FollowPrepare待機中に現側が復旧すれば同側で再開する。
- no-return前に反対側が安定成立すればMission全体を置換してShiftOutを開始する。
- no-return後、横並び、車体重複時は反対側へ切り替えない。
- actual wall contact、Emergency、solver failureは従来どおりRecoveryとなる。
- 対象パッケージのbuild/testが成功する。

## 実走確認

`make dev2`で次を確認する。

- `dynamic mission wait entered`後に即Recoveryへ落ちない。
- `opp_eval=1`がFollowPrepareでも観測される。
- `opp_current=1`ならsame-side resume、`opp_alt_ok=1`かつ`opp_ready=1`ならno-return前にatomic replacementする。
- 横並び後のside replacementが0回である。
- Pass -> Return -> Idle完遂率が`20260807-234257`の2/9を上回る。
- wall contact、Emergency、solver failureのRecoveryが抑止されていない。

## 静的検証結果

- `make autoware-build`: 成功（25 packages）。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test targets）。
- `test_v2x_overtake_core`: 885 tests、失敗0。dynamic waitのHold／same-side再開／alternate置換／hard-fault Recovery／rear-clear Returnを追加確認。
- `git diff --check`: 成功。
- `make dev2`: 未実施。上記の実走確認項目は次回走行ログで判定する。
