# Tasklist

- [x] 最新ログのPass/Return失敗経路を特定
- [x] target-bound Pass holdの純粋ポリシーを追加
- [x] 直前MPCC軌道の物理再検証とbounded holdを実装
- [x] hold中の速度維持を実装
- [x] SafeSeparationのsoft失敗をPass内再計画へ戻す
- [x] Returnの動的距離事前検証を実装
- [x] 単体テストを追加
- [x] multi_purpose_mpc_rosをビルド・テスト

## Definition of Done

- target-boundだけの一時的不成立で即座にFollowPrepareへ移らない。
- hard faultではholdを開始・継続しない。
- Return開始時点で実行軌道が壁・横加速度制約を満たす。
- 既存テストと追加テストが成功する。

## Verification

- `make autoware-build`: 25 packages finished、成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功。
- `colcon test-result`: 1073 tests、0 errors、0 failures、0 skipped。
- `git diff --check`: 成功。
- `make dev2`: 未実施。動的効果確認は次回試走で行う。
