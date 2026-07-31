# Tasklist

- [x] 最新ログから負のループを特定
- [x] 要件・設計・対象外を記録
- [x] 直接Pass再開APIから実測closing-speed依存を削除
- [x] 低速自車の再現テストを追加
- [x] 既存横安全拒否テストを確認
- [x] `make autoware-build`（25 packages成功）
- [x] packageテスト（722 tests、失敗0）
- [x] `git diff --check`

## Static verification

- `make autoware-build`: 成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result`: 722 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: 成功

## Dynamic verification

利用者側の`make dev2`で確認する。

- 横安全条件成立後、1秒程度以内に`FollowPrepare -> Pass`へ復帰する。
- `ego_speed < target_speed`だけを理由とするguard holdが発生しない。
- pass sideを横断せず、現在・予測横離隔を維持する。
- 壁Recovery、接触、solver failureは別指標として記録する。
