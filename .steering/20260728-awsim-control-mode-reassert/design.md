# AWSIM Control Mode Reassert Design

作成日: 2026-07-28
状態: Complete

## 方針

MPC controllerは既に`/awsim/state`と実速度を取得している。このノードへ
`/awsim/control_mode_request_topic` publisherを追加し、次の有限状態で再送を管理する。

```
Inactive
  ├─ Ready ──> AwaitingMotion
  └─ Start ──> AwaitingMotion（windowを再開始）
                  ├─ retry periodごと ──> trueを再送
                  ├─ |speed| >= threshold ──> MotionConfirmed
                  └─ timeout ──> TimedOut
```

ReadyとStartのそれぞれで即時に`true`を送信する。`Spawned`、`Grounded`、`Finish`では
前sessionをresetする。再送判定にはsteady clockを使い、
ROS clockの停止・巻き戻りで要求間隔が壊れないようにする。

## 初期値

- enabled: `true`
- retry period: `0.2 s`
- timeout: `5.0 s`
- motion threshold: `0.1 m/s`

すべてROS parameterとし、`use_sim_time=true`かつ`simulation_mode=true`でのみ有効にする。

## Recoveryとの関係

`AwaitingMotion`中だけStuck detectorへ`deliberate_stop=true`相当の抑止を渡す。
これは提出ログで1.5秒後に発火したEvidence-free Recoveryを防ぐためであり、5秒timeout後や
MotionConfirmed後のRecovery判定には影響しない。既に開始済みのRecoveryを途中解除しない。

## インターフェース互換性

- 既存topic名・型を維持する。
- 評価側publisherとの複数publisher構成は現行契約どおり。
- Domain 0や`/admin/awsim/*`へは触れない。
- 評価側`aichallenge_system/`は変更しない。
