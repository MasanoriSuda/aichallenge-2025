# Tasklist

- [x] 20260809-194105の停止・横並びPassを再確認
- [x] target continuityとPass horizonの責務差を特定
- [x] pure coreへ有界prediction lease判定を追加
- [x] controllerへ最終safe predictionとlease状態を追加
- [x] horizon extension/hold失敗経路へleaseを統合
- [x] 単体テストを追加
- [x] `make autoware-build`（25 packages成功）
- [x] `colcon test --packages-select multi_purpose_mpc_ros`（25/25 targets成功）
- [x] 差分・ユーザー変更の非干渉を確認

## 動的確認

- [ ] `make dev2` で停止・横並び時に `stopped-side prediction lease started` を確認
- [ ] lease中に壁・Emergency・solver条件を緩和していないことを確認
- [ ] Reverse後は古いMissionではなくfresh Missionで再進入することを確認
