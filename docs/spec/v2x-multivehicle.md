# V2X And Multi-Vehicle

> Automotive AI Challenge 2026 の 3〜4 台同時走行と V2X 利用に関する現行方針。
>
> 確認日: 2026-07-01

## Source Of Truth

公式ルールでは、走行は 3〜4 台同時のレース形式で、利用可能なセンサに V2X 情報（他車両の位置情報）が含まれる。

公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

## Organizer Confirmation

2026-07-01 の運営回答により、障害物情報は `/v2x/vehicle_positions` のみを使用する。安全ゲートとレース挙動の障害物・他車両認識は、この topic を正とする。

LiDAR、Camera、CSV 障害物、`/aichallenge/objects` は 2026 公式障害物入力として扱わない。既存 controller の互換やローカル検証用途で残す場合も、公式 gate 通過ロジックの依存先にしない。

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
make race2
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

`make race2` は `SIM_MODE=race2` で 2 台の Autoware を起動し、両車に `use_v2x_race_behavior:=true` を渡すローカル試走用ターゲットである。既存 `make dev2` は通常の 2 台起動として残し、race behavior の有効化とは分ける。

## Race Behavior Policy

レース用の初期実装は、同一アルゴリズムの 2 台同時走行を前提に、以下の順で安全側へ倒す。

1. 前方 target が危険距離なら停止 fallback。
2. 前方 target がいるが追い越し不可なら follow speed cap で追走する。
3. 後方 target が大きく離れて追いつけない場合、先行側は catch-up wait speed cap に入り、後続が詰められる相対速度を作る。
4. 後方 target が接近している場合、先行側は yield speed cap に入り、追い越し side へ寄らず中央または現ラインを維持する。
5. 前方 target が遅く、左右の V2X 空きと壁距離が十分なら overtake lateral offset を出す。
6. 追い越し後は return blocker を見て通常ラインへ復帰し、cooldown 中は即時の抜き返しを抑制する。

Gate2 用の `use_v2x_overtake` は安全ゲート専用、レース用の `use_v2x_race_behavior` は follow / catchup_wait / yield / overtake / return を統合した多車両用として扱う。

## Design Guidelines

- 他車両・障害物位置情報は `/v2x/vehicle_positions` を唯一の正入力として扱う。
- 他車両の raw topic を直接覗く設計にしない。
- V2X 欠損、遅延、外れ値で危険制御に倒れないようにする。
- V2X を使う判断と使わない fallback を分ける。
- 追い越し判断は壁・車線・速度制限・ペナルティを同時に考慮する。
- 追い越し左右判断は、V2X の車両配置だけでなく occupancy grid 由来の左右壁距離も使う。

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
- `/v2x/vehicle_positions` の正確な message 型。
- 他車両位置情報の更新周期、遅延、座標系。
- 他車両通信の盗み見・偽データ送信とみなされる境界。
