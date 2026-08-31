# E2E Single-Vehicle Baseline Requirements

## Objective

Automotive AI Challenge 2026 End to End AI 部門向けに、既存の
TinyLidarNet 重みを使った単一車両ベースラインを再現可能な形で起動し、
3 周連続走行の可否を評価できる状態にする。

## Scope

- E2E 専用の AWSIM 単車起動経路を追加する。
- 推論時の横方向制御は 2D LiDAR を入力とする ML 出力だけで生成する。
- `/control/command/control_cmd` と提出 launch の既存契約を維持する。
- モデル形状、重み、出力の不整合を起動時または推論時に fail-fast させる。
- LiDAR 更新停止時に、最後の加速指令を保持せず停止指令へ移行する。
- 既存重みを変更せず、まず現在性能を測る。

## Constraints

- E2E 推論グラフは Camera / LiDAR / Steer Angle / Wheel Odometry /
  Gear Status 以外を入力にしない。今回のモデル入力は LiDAR のみ。
- GNSS、IMU、V2X、地図、trajectory、MPC 出力をモデル入力にしない。
- 評価基盤の Domain、admin topic、`/set_initial_pose`、result JSON 契約は変更しない。
- `aichallenge_system` は変更しない。
- `output/`、rosbag、学習生成物はコミットしない。
- Slack で共有された提出期限 2026-09-07 を作業上の期限とする。
  公式 Web ページの期限表示は WIP のため、確定値としてコードへ埋め込まない。

## Initial Findings

- 現ブランチの `make dev` は `--lidar off` であり TinyLidarNet を検証できない。
- 提出 launch の既定 controller は MPC のままで、E2E controller が自動選択されない。
- 配布重みは入力 750、出力 2、通常 TinyLidarNet の形状と一致する。
- PyTorch と NumPy 推論の差は手元確認で最大 `1.79e-7`。
- runtime は重みの欠落・shape 不一致を検出せず、scan stale watchdog もない。
- 学習データ抽出は最近傍同期の時間差を通常保存・閾値判定していない。
  これは次のデータ品質 Slice で扱う。

## Acceptance Criteria

### Static

- TinyLidarNet package と launch がビルドできる。
- 750 入力の配布重みが全 parameter 名・shape・finite 値を満たす。
- 欠落重み、shape 不一致、非 finite 入出力をテストで拒否できる。
- E2E launch では TinyLidarNet だけが final control publisher になる。

### Dynamic

- E2E 単車モードで `/sensing/lidar/scan` が継続 publish される。
- `/control/command/control_cmd` が継続 publish され、全値が finite。
- LiDAR を止めた場合、設定 timeout 内に停止指令へ移行する。
- 固定スタートで 3 周連続走行し、result とログを保存できる。
- 失敗時は「node 未起動」「scan 欠損」「推論異常」「走行逸脱」を区別できる。
