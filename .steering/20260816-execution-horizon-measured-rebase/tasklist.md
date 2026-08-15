# Tasklist

- [x] 現行 rolling refresh と最新ログの failure boundary を確認
- [x] measured-state rebase retry の pure policy を追加
- [x] rolling refresh 評価を局所的に共通化
- [x] 通常 stitch 不成立時の measured-state rebase を追加
- [x] local/cloud config と起動ログを更新
- [x] unit test を追加
- [x] `make autoware-build` を実行
- [x] `test_v2x_overtake_core` を実行
- [x] tasklist と検証結果を更新
- [x] 変更をコミット

## Definition of Done

- 通常 stitch が成立する場合は従来経路を採用する。
- 通常 stitch が不成立でも、hard fault がなく measured rebase が全検証を通れば採用する。
- hard fault / target discontinuity では measured rebase を試さない。
- 両候補が不成立なら active path を保持する。
- ROS インターフェースと評価基盤の変更がない。

## Verification

- `make autoware-build`: 25 packages successful
- `test_v2x_overtake_core`: 653/653 passed
- 最終リファクタ後の追加2 test: 2/2 passed
- `config.yaml` / `config_for_cloud.yaml`: YAML parse successful
- 動的効果確認: `make dev2` 実走待ち
