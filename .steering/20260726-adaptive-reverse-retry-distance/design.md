# Design

## 連続失敗の定義

`RejoinComplete` を観測した時点から、Normal中の正方向移動量を積算する。

- 積算が `adaptive_retry_reset_forward_distance_m` 未満のまま次のRecoveryが始まる:
  連続失敗としてretry levelを1増やす。
- 積算が設定値以上:
  復帰成功としてretry levelを0に戻し、次回を初回Recoveryとして扱う。

時間だけでは低速rejoinを誤って成功扱いするため、リセット条件は前進距離とする。

## 目標距離

```text
target = min(base_target * multiplier ^ retry_level, maximum_target)
```

初期設定:

- `adaptive_reverse_retry_enabled: true`
- `adaptive_reverse_retry_multiplier: 2.0`
- `adaptive_reverse_retry_max_distance_m: 4.0`
- `adaptive_retry_reset_forward_distance_m: 5.0`

fast rejoinの基準0.8 mなら `0.8 -> 1.6 -> 3.2 -> 4.0 m`、
通常基準2.0 mなら `2.0 -> 4.0 -> 4.0 m` となる。

## 安全上限

予測停止距離を含むhard limitと目標距離を分離する。

- adaptive target上限: 4.0 m
- hard reverse distance: 5.0 m
- reverse duration: 6.0 s

候補選択時はadaptive target全体の静的rolloutとV2X corridorを評価する。
安全な4 m corridorがない場合は、距離を延長して後退しない。

## 状態保持

純粋C++の `AdaptiveReverseDistanceTracker` が以下を保持する。

- retry level
- RejoinComplete後の再発監視中か
- Normal中の累積前進距離

ROS adapterはRecovery開始・RejoinComplete・Normal前進量をtrackerへ通知し、
trackerが返すactive targetをrollout、制動、escape確認へ共通使用する。

## 互換性

- ROS interface: 変更なし
- 評価成果物: 変更なし
- simulation-only guard: 維持
- 機能無効時: retry level 0相当の従来距離
