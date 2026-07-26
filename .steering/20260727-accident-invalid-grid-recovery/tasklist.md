# Tasklist

- [x] 最新D3ログから復帰不能シーケンスを特定
- [x] rosbagの自己位置と地図範囲を照合
- [x] 候補未評価とReverse-only latchの組合せを特定
- [x] Rear wall時にsolver Reverse-onlyを解除
- [x] 候補未評価の診断値を`not_evaluated`へ変更
- [x] 方向選択ポリシーの単体テストを追加
- [x] 対象packageをビルド・テスト
- [x] 実走確認項目を記録

## 検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 24/24 test targets成功
- `colcon test-result`: 632 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 問題なし

## 実走確認項目

1. 事故後に後方の壁へ接近した際、
   `Stuck recovery released Reverse-only latch: explicit Rear wall requires Forward escape`
   が一度出ること。
2. 続くRecovery debugで`wall=rear`かつ`direction=Forward`、
   `primitive=forward_straight`が選択されること。
3. `FORWARD_MANEUVER -> LOW_SPEED_REJOIN -> NORMAL`へ復帰すること。
4. 同じ姿勢で`maneuver_direction_unknown`とSafeStopの再試行を繰り返さないこと。
5. 前方rolloutが実際に塞がっている場合は、具体的な静的/V2X棄却理由で
   SafeStopを維持し、盲目的に前進しないこと。

## Definition of Done

- Rear wall時は、solver fallbackかつheading error大でもForward候補を選べる。
- 候補未評価時は`invalid_grid`ではなく`not_evaluated`と記録される。
- V2X・ギア・距離・速度・simulation-onlyゲートを維持する。
- `multi_purpose_mpc_ros`のビルドと単体テストが成功する。
