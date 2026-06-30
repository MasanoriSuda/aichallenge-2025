# V2X And Multi-Vehicle

> Automotive AI Challenge 2026 の 3〜4 台同時走行と V2X 利用に関する現行方針。
>
> 確認日: 2026-07-01

## Source Of Truth

公式ルールでは、走行は 3〜4 台同時のレース形式で、利用可能なセンサに V2X 情報（他車両の位置情報）が含まれる。

公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

## Current Local Contract

現行ローカル設計では、車両ごとに ROS_DOMAIN_ID を分ける。

| Domain | 役割 |
|---|---|
| `0` | AWSIM シミュレータ本体と管理ノード |
| `1..N` | 車両ごとの Autoware インスタンス |

現行ローカル docs では、クロスドメイン通信は `/v2x/vehicle_positions` と `v2x_msgs` に限定し、`domain_bridge` は復活させない方針。

## Local Commands

```bash
make dev2
make dev3
make dev4
```

確認:

```bash
make ps
```

コンテナ内確認:

```bash
ros2 topic list
ros2 topic echo --once /v2x/vehicle_positions
```

## Design Guidelines

- 他車両位置情報は主要入力として扱う。
- 他車両の raw topic を直接覗く設計にしない。
- V2X 欠損、遅延、外れ値で危険制御に倒れないようにする。
- V2X を使う判断と使わない fallback を分ける。
- 追い越し判断は壁・車線・速度制限・ペナルティを同時に考慮する。

## Evidence To Keep

- `/v2x/vehicle_positions` の publish rate
- 自車 localization
- 他車両との相対距離
- control command
- penalty events
- collision / wall contact の有無

## Organizer Confirmation Needed

以下は 2026 公式インターフェースとの整合確認が必要。

- 公式評価環境で `domain_bridge` を使うかどうか。
- V2X message の正確な型と topic 名。
- 他車両位置情報の更新周期、遅延、座標系。
- 参加者が subscribe してよい V2X topic の範囲。
- 他車両通信の盗み見・偽データ送信とみなされる境界。
