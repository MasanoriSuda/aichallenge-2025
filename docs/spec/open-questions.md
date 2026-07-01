# Open Questions

> Automotive AI Challenge 2026 ベース化で、公式ページ更新または運営確認が必要な事項。
>
> 確認日: 2026-07-01

## Purpose

2026 公式仕様と現行 2025 由来実装の差分を、実装へ固定する前に管理する。
この文書にある項目は、確認が取れるまで `docs/interface/` の確定契約として扱わない。

## Confirmed

| 確認日 | 項目 | 確定内容 | 反映先 |
|---|---|---|---|
| 2026-07-01 | 障害物情報の入力 | 運営回答により、障害物情報は `/v2x/vehicle_positions` のみを使用する。LiDAR、Camera、CSV、`/aichallenge/objects` は 2026 公式障害物入力として扱わない。 | `docs/spec/safety-gates.md`, `docs/spec/v2x-multivehicle.md`, `docs/interface/participant-interface.md`, `docs/spec/mpc-integration.md` |

## High Priority

| 項目 | 現行ローカル | 2026 公式/推定 | 必要な確認 |
|---|---|---|---|
| クロスドメイン通信 | `domain_bridge` は廃止、`/v2x/vehicle_positions` のみ | 公式インターフェース側では `domain_bridge` 記載がある可能性 | 2026 評価環境で `domain_bridge` が必要か |
| LiDAR topic | `/scan` | 2026 障害物入力では使わない | `tiny_lidar_net` を継続する場合の正式 topic 名と型 |
| Camera topic | `/image_raw` | 2026 障害物入力では使わない | `pilot_net` を継続する場合の正式 topic 名と型 |
| V2X schema | `/v2x/vehicle_positions` | 障害物情報はこの topic のみ | 正式 message 型、座標系、周期、欠損時の扱い |
| Safety gates | `make gate1/2/3` は `--safety-gate 1/2/3` を起動 | `gate1`: 障害物停止、`gate2`: 追い越し、`gate3`: 車線維持 | 公式合否基準、シナリオ詳細、スタック時復帰 |
| Result schema | 現行 `result-summary.json` v2 / `dN-result-details.json` v3 | 2026 レース順位・ペナルティ対応 | schema 変更の有無 |
| Submission | `submit/aichallenge_submit.tar.gz` | 公式プラットフォーム提出 | 提出先、回数制限、評価環境 |

## Medium Priority

| 項目 | 確認したいこと |
|---|---|
| タイムアップ | 公式ページ上は 10 分予定だが未定扱い。確定値が必要。 |
| スタート位置 | ランダムまたは過去戦績予定。評価再現性への影響確認が必要。 |
| ブーストアイテム | 利用できる入力/出力、制御への影響、評価ログへの出方。 |
| ハンディキャップ | 順位による加速度・速度差の仕様。 |
| スタック時復帰 | SIM 決勝・実機決勝で手動復帰できるか。 |
| コードチェック | 禁止事項の具体的検査範囲。 |
| 実車停止手順 | オペレータ停止、遠隔操作、緊急停止の公式手順。 |

## How To Resolve

1. 公式ページ更新を確認する。
2. 更新で解決しない場合は運営に問い合わせる。
3. 回答をこの文書に追記する。
4. 確定した内容だけ `docs/spec/` または `docs/interface/` に移す。
5. 契約変更を伴う場合は、実装変更前に migration note を書く。
