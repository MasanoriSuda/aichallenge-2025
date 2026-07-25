# Tasklist

- [x] 最新ログのRecovery遷移と所要時間を確認
- [x] 現行stepwise候補選択と停止距離reserveを確認
- [x] 要件・設計を記録
- [x] fast continuous Reverse設定を追加
- [x] clear footprint時の連続rollout選択を実装
- [x] 前進可能時の早期escape目標を実装
- [x] stopping reserveによる連続Reverse内制動を実装
- [x] Reverse加速度・速度・制動設定を更新
- [x] 単体テストを追加
- [x] 対象packageをビルド・テスト
- [x] 実走確認項目を記録

## Definition of Done

- clearな2 m後退corridorがある場合、0.4 mごとにDriveへ切り替えない。
- 前進rejoin corridorがclearなら、最小距離後に一度だけDriveへ切り替える。
- 接触中、後方blocked、V2X不完全、collision worseningでは既存の安全停止を維持する。
- Reverse加速度指令は `1.0 m/s^2` 以下とする。
- `multi_purpose_mpc_ros` のビルド・テストが成功する。

## Verification

- `make autoware-build`: 25 packages succeeded
- `colcon test --packages-select multi_purpose_mpc_ros`: 626 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: passed

## 実走確認項目

1. 起動ログで `Stuck recovery continuous Reverse: enabled`、`speed<=2.00`、`accel=1.00` を確認する。
2. 後方がclearな最初の候補で `stepwise=0, continuous=1` になることを確認する。
3. clearな間は短距離ごとの `SHIFT_TO_DRIVE -> SHIFT_TO_REVERSE` が反復しないことを確認する。
4. `REVERSE_MANEUVER` のまま `reason=reverse_escape_braking` で制動し、その後のDrive切替が一度だけになることを確認する。
5. 前進rejoinもclearなら後退約0.8 m、未成立なら約2.0 mを目標にすることを確認する。
6. stuck確定から最初のReverse開始までの時間、総後退距離・時間、最高速度、停止overshootを記録する。
7. 接触cellが残る場合は、脱出方向を作るまで従来のstepwise動作が残ることを確認する。
