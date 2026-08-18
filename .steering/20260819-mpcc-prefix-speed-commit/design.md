# Design

## 原因

worker が時刻 A の ego/target 速度で候補を生成した後、live callback は時刻 B の速度から
`min(ego_speed, target_speed)` を再計算していた。候補の `predicted_minimum_ego_speed_mps`
は時刻 A の rollout に属するため、異なる時点の値を比較すると fresh な候補でも
`minimum-speed-insufficient` になり得る。

## 採用契約

`OvertakeMissionCandidate` に計画時の最低速度要求値を保存する。

```text
candidate predicted minimum speed
        >=
planning snapshot minimum-speed requirement - tolerance
```

計画時要求値が存在しない旧形式・テスト候補だけは、live 要求値を使用する。
live callback は引き続き現在状態から tracking MPCC を組み直し、wall/target bounds、
result age、target/generation/phase を再検証する。したがって、この変更は古い制御入力を
再生せず、非同期 tactical metadata の時間基準だけを一致させる。

## 局所リファクタリング

- snapshot-consistentな最低速度判定を pure function に分離する。
- progressive prefix と complete cross-side Mission の admission が同じ helper を使う。
- 拒否ログには `predicted/planned/live` を分けて出し、再発時に時間不整合か真の不足かを判別する。

## 非対象

- wall・target余裕の緩和。
- async workerの別プロセス化。
- Recovery/Reverseの変更。
- MPCC solver・重み・速度上限の変更。
