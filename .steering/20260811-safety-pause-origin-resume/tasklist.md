# Tasklist

- [x] 現行のpause再開guardと速度所有権を確認する
- [x] 要件・安全境界を記録する
- [x] pause causeを状態へ追加する
- [x] origin-aware resume resolverをpure coreへ追加する
- [x] Behavior ownershipへ統合する
- [x] OvertakeLine再開遷移へ統合する
- [x] 単体テストを追加する
- [x] build/testを実行する
- [x] 差分確認とコミットを行う

## 動的確認項目

- `Pass/ShiftOut -> FollowPrepare, cause=SafetyBrake`の回数
- SafetyBrake解除から`FollowPrepare -> ShiftOut/Pass`までの時間
- FollowPrepare timeout 4秒へ到達した回数
- pause前後の最低速度とtarget相対距離
- wall/solver Recovery、接触、反対側切替の増加有無

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test suites成功
- `colcon test-result`: 963 tests、0 errors、0 failures、0 skipped
- param / launch / ROS 2 interfaceの変更なし
- 既存front-hazard holdは0.25秒であり、4秒のFollowPrepare timeoutとは分離されていることを確認
