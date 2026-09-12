# V2X And Multi-Vehicle

> Automotive AI Challenge 2026 の 3〜4 台同時走行と V2X 利用に関する現行方針。
>
> 確認日: 2026-09-09（V2X提供情報・遅延と後方情報の用途を再確認）

## Source Of Truth

公式ルールでは、走行は 3〜4 台同時のレース形式で、利用可能なセンサに V2X 情報（他車両の位置情報）が含まれる。

公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

2026-09-09確認: [公式シミュレーター仕様](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html)は
`/v2x/vehicle_positions`と、車両ごとに100〜200msで変動する伝送遅延を明記している。
配列stampだけでの速度計算には注意が必要である。正確なmessage型・座標系・各車両stampの
意味は引き続き確認対象。姿勢・操舵・公開計画の提供は確認できていない。

[公式禁止事項](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html)は、
後方V2Xを使う進路妨害や不要な減速を禁止し、安全確認のための参照は認めている。
後方車の検査が失敗した事実だけで不要な減速を正当化せず、原因と安全上の必要性を記録する。
相手のraw topicや未観測姿勢で形状を狭める設計にはしない。

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

## Explicit Empty-World Observation

AWSIMのnative V2X fanoutは、単車シナリオでpeerが存在しない場合に
`/v2x/vehicle_positions`をpublishしない。NoDataを「他車両なし」と推測すると、多車両時の
V2X通信断でも空コースとして走行するため、controller側ではNoDataをfail-closeのまま扱う。

ローカル起動層はシナリオ車両数を`AIC_VEHICLE_COUNT`としてAutoware launchへ渡す。
simulationかつ車両数1の場合だけ、各vehicle Domain内の
`single_vehicle_empty_v2x_publisher`がtimestamp付きの空`V2XVehiclePositionArray`をpublishする。
`make dev2`以上ではこのproducerを起動せず、AWSIM native V2Xだけを使用する。

2026-09-13: MPCC Recoveryも、この宣言値を追加parameter `recovery_vehicle_count`で
受け取る。既定は環境変数`AIC_VEHICLE_COUNT`、未指定時は0（台数不明）。simulationでのみ、
自車除外配列ならN−1、明示自車IDを含む配列ならNの既知IDを要求する。
単車の空配列も、現在のRecovery epochで受信した新鮮で有効なmessageが必要である。
NoData、期限切れ、不正時刻、欠落IDを台数宣言で補完しない。台数不明時の空配列も不完全とする。
旧force-motionによる不完全V2X・自車不一致・peer障害の無視は廃止する。
[参加者契約](../interface/participant-interface.md)を参照。2025由来のローカル暫定であり、
2026公式の台数・V2X契約との同等性はTBD。

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
- V2X message の正確な型（topic名は公式シミュレーター仕様で確認済み）。
- 他車両位置情報の更新周期、座標系、各車両stampの意味と実環境での遅延特性。
- 参加者が subscribe してよい V2X topic の範囲。
- 他車両通信の盗み見・偽データ送信とみなされる境界。
