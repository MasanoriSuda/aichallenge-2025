# Tasklist

- [x] 最新ログから反復経路を特定する
- [x] 修正範囲と非対象を記録する
- [x] Mission保持禁止フラグを追加する
- [x] Mission budget AbortからRecoveryへフラグを引き継ぐ
- [x] Recovery保持判定とログを更新する
- [x] core単体テストを追加する
- [x] 対象packageをbuild/testする
- [x] 動的確認項目を記録する

## Static verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 test targets）
- `colcon test-result --verbose`: 931 tests、0 errors、0 failures、0 skipped
- 既存の`joycon_contract_guard/package.xml`欠損に関する集計warningあり。今回の変更とは無関係。

## Dynamic verification（次回make dev2）

- `same-target Mission total budget expired`が同一Missionで1回だけ出ること。
- 直後のRecovery完了時に`committed pass mission retained`が出ないこと。
- `Recovery -> Idle`の理由に`after terminal Mission abort`が出ること。
- budget expiry起点の`Recovery -> FollowPrepare -> Recovery`反復が0回であること。
- 通常の短期Recoveryでは従来どおり必要時にMissionを保持できること。
