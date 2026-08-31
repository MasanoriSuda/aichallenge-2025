# E2E Single-Vehicle Baseline Task List

- [x] Root / participant / evaluation contract を確認
- [x] 現 E2E 資産と upstream `develop_e2e` を差分監査
- [x] 配布重みの key / shape と NumPy-PyTorch parity を机上確認
- [x] E2E 公式・単車 simulator mode を追加
- [x] E2E controller selection を参加者 launch に接続
- [x] TinyLidarNet import / model artifact 契約を修正
- [x] scan stale watchdog と inference fail-safe を追加
- [x] core / launch contract test を追加
- [x] E2E 仕様ドキュメントを追加
- [x] `make autoware-build` と package test
- [x] `make e2e-single` smoke test
- [x] topic / command / watchdog を確認
- [x] 3 周連続 Gate を実施（未達原因を分類）
- [x] 走行結果を記録し、次 Slice を決定

## Definition of Done

- 静的契約テストとビルドが成功する。
- 既存 TinyLidarNet が E2E 単車モードで明示的に起動する。
- 3 周結果、または失敗分類に必要なログが得られる。
- 変更と検証結果をコミットする。

## Result

- 0.6 m/s² run: 103.64秒、90.66秒の2周後、3周目で壁スタック。
- 最長低速区間: 184.55秒。開始はbag開始239.80秒後。
- スタック直前10秒の最小LiDAR距離中央値: 0.159 m。
- スタック後、右側に空間があってもsteeringは約0.03 radへ縮小。
- 0.3 m/s² A/B: スタートライン前で停止し、発進baselineとして不採用。
- LiDAR: 750点、20 Hz、stale 0。推論: おおむね2〜4 ms。
- 次Slice: 教師bag同期契約、run-level split、failure/recovery data収集。
