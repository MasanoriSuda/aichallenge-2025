# E2E Single-Vehicle Baseline Design

## Boundary

評価インフラは既存のまま維持し、参加者側だけを E2E 用に構成する。

```text
AWSIM 2D LiDAR
  -> /sensing/lidar/scan
  -> TinyLidarNet NumPy inference
  -> steering_tire_angle
  -> AckermannControlCommand
  -> /control/command/control_cmd
```

GNSS / localization / planning node が評価ハンドシェイクの都合で起動していても、
TinyLidarNet controller はそれらを subscribe しない。モデルの入力グラフと評価基盤を
分けて監査する。

## Launch Strategy

- `e2e-single.sh`: 1 vehicle、NPC 0、3 laps、LiDAR on、Camera/IMU/V2X off。
- 同梱AWSIMはGNSS publisherがないとReady/Groundedへ到達しないため、ローカル
  E2E modeではGNSSを初期化基盤専用に有効化する。TinyLidarNetのROS graphには
  GNSS subscriptionを持たせない。
- `e2e.sh`: 公式 upstream 由来の練習条件を保持する。
- `e2e-final.sh`: 公式 upstream 由来の決勝参考条件を保持する。
- E2E ブランチの参加者 entry launch は `tiny_lidar_net` を既定 controller とする。
- `/control/command/control_cmd` の topic と message type は変更しない。

## Runtime Contract

### Model loading

重みは全 parameter について次を検証する。

- 必須 key がすべて存在する。
- 余分な key がない。
- shape が runtime architecture と一致する。
- dtype を float32 として使用でき、全値が finite。

不一致をランダム初期値で補完しない。node 起動を失敗させ、ログに理由を出す。

### Inference

- 空 scan を拒否する。
- NaN / -Inf は 0、+Inf は max range として学習側と同じ前処理を行う。
- 出力 shape、finite、物理 steering 上限を検証する。
- 推論例外時は加速指令を再利用せず、停止指令を publish する。

### Sensor watchdog

- ROS clock で最後の正常 scan 受信時刻を保持する。
- timeout 超過時は負加速度、steering 0 の停止指令を反復 publish する。
- scan 復帰後は通常推論へ戻す。
- stale 遷移と復帰は一度だけ明示ログを出す。

## Validation Order

1. core unit tests
2. launch/source contract tests
3. package build and test
4. E2E single mode node/topic smoke test
5. 3-lap run
6. result/log classification

E2E走行bagには、出力指令だけでなく推論を再現できるよう
`/sensing/lidar/scan` を保存する。

既存重みで単車走行が成立しない場合、すぐ閾値調整をせず、scan 分布・教師データ・
steering 出力分布を確認してから再学習 Slice を切る。

配布設定0.3 m/s²とのA/Bを行った結果、0.3ではスタートライン前で停止し、0.6では
2周を連続完了した。よって現AWSIMの暫定baselineは0.6 m/s²とする。ただし3周目の
壁スタックは解消していないため、これ以上の加速度調整ではなくfailure/recovery dataを
含む再学習で解消する。
