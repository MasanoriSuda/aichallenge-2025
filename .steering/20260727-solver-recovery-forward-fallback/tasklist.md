# Tasklist

- [x] 最新P1ログから停止開始点とSafeStop反復を特定
- [x] Reverse候補とForward rejoinの安全判定結果を照合
- [x] Forward fallback解禁条件を設計
- [x] 解禁条件をpure policyとして実装
- [x] solver Reverse-onlyに明示的な解禁を反映
- [x] wall=None時のForward候補選択を実装
- [x] 回帰テストを追加
- [x] ビルド・対象packageテスト
- [x] 実走確認項目を記録

## 検証結果

- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result`: 634 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 問題なし

## 実走確認項目

1. 初回Recoveryでは従来どおり`reverse_only=1`でReverse候補を評価すること。
2. Reverse候補が静的collisionでSafeStopへ入った後、次のログが一度出ること。
   `Stuck recovery unlocked solver Forward fallback after blocked Reverse`
3. 続くaggressive retryで`forward_fallback=1`になること。
4. 次の候補選択で`direction=Forward`と
   `primitive=forward_straight|forward_left|forward_right`のいずれかになること。
5. `FORWARD_MANEUVER -> LOW_SPEED_REJOIN -> NORMAL`へ復帰すること。
6. Forward rolloutまたはV2Xが塞がる場合は、具体的な棄却理由で停止を維持すること。

## Definition of Done

- 最初のsolver recoveryはReverse優先を維持する。
- Reverse候補が静的collisionで全滅した場合だけ、SafeStop後にForwardを評価する。
- Forwardの静的/V2X/Boost/ギア/距離/速度ゲートを維持する。
- 同一姿勢で`maneuver_direction_unknown`を永久反復しない。
- `multi_purpose_mpc_ros`のビルドと単体テストが成功する。
