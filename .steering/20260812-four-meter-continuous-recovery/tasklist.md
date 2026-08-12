# Tasklist

- [x] 最新試走と現行Recovery経路を確認する
- [x] requirements/designを作成する
- [x] 初回4 m、4 -> 8 m設定を反映する
- [x] improving-contact 4 m continuous preflightを実装する
- [x] RecoveryIncidentLedgerを実装・統合する
- [x] aggressive retryでadaptive targetを8 mへ進める
- [x] runtime pose-jump guardを物理上限対応にする
- [x] gear requestを最大3回再送可能にする
- [x] 単体テストを追加する
- [x] Docker内build/testを実行する
- [x] 動的 `make dev2` の確認項目を記録する

## Definition of Done

- 初回ログにReverse target 4.0 mが出る。
- clear/improving-contact 4 m rolloutでは`continuous=1`になる。
- 同一incidentのaggressive retry後もincident距離・時間・retryが単調増加する。
- 最初のaggressive retry後のReverse targetが8.0 mになる。
- 2.0 m/s、0.25 rad、25--50 msの正常運動をpose jumpにしない。
- 0.5 m級teleportはrejectする。
- 初回gear要求drop後、0.2秒間隔で最大3回まで再送する。
- package build/testが成功する。

## 動的確認

- `make dev2`
- stuck検知から最初の物理移動までの時間
- initial target / continuous / selected primitive
- incident elapsed / total/reverse/forward distance / retries / gear requests
- collision_worseningとpose_jumpの有無
- rejoin_completeまでの時間
- Rejoin後5 mでincident resetされること

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result`: 994 tests、0 errors、0 failures、0 skipped
- `make dev2`: 未実施。上記「動的確認」をユーザー試走で確認する。
