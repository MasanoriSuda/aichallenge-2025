# Localization Scope 要件

## 目的

Automotive AI Challenge 2026 参加者が、GNSS・IMU・車両状態・自己位置推定・
trajectory の rosbag をオフラインで可視化し、単一走行の状態確認と 2 走行の
Baseline/Candidate 比較を行えるツールを提供する。

## 配置

`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope/`

Kaleidoscope と同様に、運営提供の AI Challenge Docker イメージと、この
リポジトリのディレクトリ構成を維持した checkout を対応環境とする。

## MVP 機能

1. 単一の `.mcap` または rosbag ディレクトリを読み込む。
2. 利用者が記入する最小の `run-metadata.json` を検証する。
3. 未指定の trajectory/config/topic は現行リポジトリの既定値で補完する。
4. trajectory CSV、実行時 trajectory、GNSS pose、EKF pose を区別する。
5. trajectory に対する横偏差・方位偏差、GNSS-EKF 差、速度帯別統計を算出する。
6. 単一走行の `report.html` と `summary.json` を出力する。
7. Baseline/Candidate の 2 走行を比較し、改善・悪化・実質差なし・判定不能を表示する。
8. 取得できない topic や metadata は推測せず、警告または `null` として扱う。
9. run 一覧からブラウザ上で単一走行または2走行を選択できる。

## 利用者が記入する情報

- AWSIM version
- run label / test type
- 目標速度・速度 profile
- 標準構成と異なる trajectory/config/topic
- 任意の比較 group、variant、notes

## 自動取得する情報

- bag の topic/type/count/rate/duration
- 実速度帯
- trajectory/config の解決パスと SHA-256
- 取得可能な Git commit と dirty 状態
- 解析 tool version

## 制約

- simulator 真値を前提にしない。
- trajectory は真値ではなく目標経路として扱う。
- GNSS-EKF 差と trajectory 追従偏差を混同しない。
- `output/`、rosbag、既存設定を変更しない。
- Plotly、pandas、NumPy を必須依存にしない。
- ROS message の読み出しは運営 Docker 内の `rosbag2_py` 等を利用する。
- 2025 記事で観測された固定バイアスや遅延値を 2026 の閾値に流用しない。

## Definition of Done

- metadata template を生成できる。
- 現行の最新 MCAP から、不足 topic 警告付きの単一 HTML/JSON を生成できる。
- 合成データから単一レポートと 2 走行比較レポートを生成できる。
- core の単体テストが通る。
- README に前提、コマンド、metadata、指標の意味と限界が記載されている。
