# Open Questions

> Automotive AI Challenge 2026 ベース化で、公式ページ更新または運営確認が必要な事項。
>
> 確認日: 2026-07-12

## Purpose

2026 公式仕様と現行 2025 由来実装の差分を、実装へ固定する前に管理する。
この文書にある項目は、確認が取れるまで `docs/interface/` の確定契約として扱わない。

## High Priority

| 項目 | 現行ローカル | 2026 公式/推定 | 必要な確認 |
|---|---|---|---|
| クロスドメイン通信 | `domain_bridge` は廃止、`/v2x/vehicle_positions` のみ | 公式インターフェース側では `domain_bridge` 記載がある可能性 | 2026 評価環境で `domain_bridge` が必要か |
| LiDAR topic | `/scan` | `/sensing/lidar/scan` の可能性 | 正式 topic 名と型 |
| Camera topic | `/image_raw` | `/sensing/camera/image_raw` の可能性 | 正式 topic 名と型 |
| V2X topic | `/v2x/vehicle_positions` | V2X 他車両位置情報 | 正式 topic 名、型、座標系、周期 |
| Safety gates | `make gate1/2/3` | 障害物停止、NPC 追い越し、車線維持 | gate 対応関係と合否基準 |
| Result schema | 現行 `result-summary.json` v2 / `dN-result-details.json` v3 | 2026 レース順位・ペナルティ対応 | schema 変更の有無 |
| Submission | `submit/aichallenge_submit.tar.gz` | 公式プラットフォーム提出 | 提出先、回数制限、評価環境 |

## Medium Priority

| 項目 | 確認したいこと |
|---|---|
| タイムアップ | 公式ページ上は 10 分予定だが未定扱い。確定値が必要。 |
| スタート位置 | ランダムまたは過去戦績予定。評価再現性への影響確認が必要。 |
| ブーストアイテム運用 | 公開I/Fと効果は確認済み。オンライン予選で実際に付与される回数、評価ログへの出方は未確認。 |
| ハンディキャップ | 順位による加速度・速度差の仕様。 |
| スタック時の手動復帰 | SIM 決勝・実機決勝で「手動操作による復帰が許される（未定）」の確定条件、操作者、ペナルティ、復帰後の競技再開条件。SIM予選は提出コードの自動走行のため、手動介入があるか。 |
| 自律REVERSE復帰 | 参加者コードが通常のgear / control topicで自律後退してスタックから復帰することは、SIM予選・SIM決勝・実機決勝の各環境で許可されるか。許可される場合の最大速度、距離、時間、試行回数、逆走・妨害・ペナルティ判定は何か。 |
| REVERSE制御意味 | AWSIMで `GearCommand=REVERSE(20)` の間、`AckermannControlCommand.longitudinal.acceleration` の正負は駆動・制動のどちらを意味するか。公式に未使用とされる `longitudinal.speed` を後退判定に参照する実行環境があるか。 |
| Gear acknowledgement | `GearCommand` 送信後、駆動指令前に対応する `GearReport` を必ず待つ必要があるか。切替時間、再送方法、timeout、不一致時の規定動作は何か。 |
| コードチェック | 禁止事項の具体的検査範囲。 |
| 実車停止手順 | オペレータ停止、遠隔操作、緊急停止の公式手順。 |

## Resolved 2026 Interfaces

### Gear（2026-07-12確認）

- 指令: `/control/command/gear_cmd`、`autoware_auto_vehicle_msgs/msg/GearCommand`。
- 状態: `/vehicle/status/gear_status`、`autoware_auto_vehicle_msgs/msg/GearReport`。
- gear値: `1=NEUTRAL`、`2=DRIVE`、`20=REVERSE`。
- AWSIMの `/control/command/control_cmd` は `longitudinal.speed` を使用せず、
  `longitudinal.acceleration` を目標加速度として使用する。
- gear I/OはAWSIMと各車両Domain N内で使用する。Domain 0の
  `/admin/awsim/reset` はシミュレーション全体管理用であり、車両ごとのgear変更や
  スタック復帰の代替手段にしない。
- 自律後退によるスタック復帰の競技上の許可、REVERSE中の加速度符号、
  gear acknowledgementの手順は未確定であり、本節の解決済みtopic契約には含めない。
- 参照: [公式インターフェース](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html)。

### AWSIM Boost（2026-07-11確認）

- 状態: `/awsim/status`、`std_msgs/msg/Float32MultiArray`。index 5=`boostRemaining`、6=`isBoosting`。
- 指令: `/awsim/cmd`、`std_msgs/msg/Float32MultiArray`。index 0を`0.0`から`1.0`以上へ立ち上げると発動し、その後`0.0`へ戻す。
- 効果: 加速度`+0.5 m/s²`、10秒。
- 使用可能回数: AWSIMの`--boosts`で設定可能。既定5だが現行ローカルdev/evalは2であり、オンライン運用値は未確認。
- 参照: [公式インターフェース](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html)、[公式シミュレーター仕様](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/simulator.html)。

## How To Resolve

1. 公式ページ更新を確認する。
2. 更新で解決しない場合は運営に問い合わせる。
3. 回答をこの文書に追記する。
4. 確定した内容だけ `docs/spec/` または `docs/interface/` に移す。
5. 契約変更を伴う場合は、実装変更前に migration note を書く。
