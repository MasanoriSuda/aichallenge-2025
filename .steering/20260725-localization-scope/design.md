# Localization Scope 設計

## 全体構成

```text
MCAP / rosbag directory
run-metadata.json
trajectory CSV
config files
        |
        v
bag_reader -> normalized RunData -> metrics
                                  -> single report
                                  -> two-run comparison
```

ROS 2 node ではなく、Kaleidoscope と同じオフライン Python package とする。
解析ロジック、bag I/O、HTML 描画を分離し、解析ロジックは ROS 非依存でテストする。

## ファイル構成

```text
tools/localization_scope/
├── README.md
├── README.ja.md
├── LICENSE
├── pyproject.toml
├── examples/run-metadata.example.json
├── schemas/run-metadata.schema.json
└── localization_scope/
    ├── __init__.py
    ├── __main__.py
    ├── cli.py
    ├── metadata.py
    ├── repository.py
    ├── trajectory.py
    ├── bag_reader.py
    ├── analysis.py
    └── report.py
```

## Metadata

利用者入力の `run-metadata.json` と、自動解決結果を含む `run-manifest.json` を
分ける。schema version は `1.0` とし、未知の optional field は保持する。

trajectory/config は `requested_path` と `resolved_path` を区別する。解析時の
ファイル内容を識別できるよう SHA-256 を manifest に保存する。実行時に変化する
trajectory は topic として別系列で扱う。

## 正規化データ

各系列は bag record timestamp を秒へ変換し、共通の `Sample(t, values)` へ
正規化する。message header stamp は取得可能な場合に別値として保存する。

主要 series:

- `ekf_pose`: x, y, z, yaw, vx, covariance
- `gnss_pose`: x, y, z, yaw, covariance
- `gnss_fix`: latitude, longitude, covariance, status
- `imu_raw` / `imu_corrected`: yaw_rate, acceleration
- `vehicle_velocity`: longitudinal velocity, heading rate
- `steering`: actual steering
- `control`: commanded steering/speed/acceleration
- `runtime_trajectory`: x, y, yaw, velocity

## 軌道対応付け

CSV trajectory の各 segment へ点を射影し、最近傍距離だけでなく以下を求める。

- segment progress `s`
- signed cross-track error
- along-segment position
- reference yaw
- yaw error

周回コースの自己交差や hairpin の誤対応を減らすため、点と segment の距離に加え
heading 差を tie-break に使う。MVP では全 segment 探索とし、性能上必要になった
場合だけ空間 index を追加する。

## 単一レポート

- run/環境/入力 manifest
- topic availability と rate
- trajectory/GNSS/EKF XY overlay
- speed time series
- trajectory progress に対する cross-track error
- GNSS-EKF distance
- speed band 別 metric table
- 警告と解析限界

## 2 走行比較

Baseline/Candidate の manifest 差分を先に表示する。比較可能な metric のみ delta を
計算し、metric ごとの tolerance によって分類する。

```text
lower-is-better metric:
  candidate < baseline - tolerance -> 改善
  candidate > baseline + tolerance -> 悪化
  otherwise                         -> 実質差なし

missing / incompatible              -> 判定不能
```

trajectory/config/AWSIM version が同時に変わっている場合は、結果を表示しつつ
単一要因へ帰属できない旨を警告する。

## Run catalog

親ディレクトリ以下の`run-metadata.json`を探索し、各runを一度解析する。
`runs-index.json`にはmanifest、summary、2 run比較結果を保存し、`catalog.html`には
表示用plot dataも埋め込む。ブラウザ上でSingleまたはBaseline/Candidateを選択し、
外部serverやROS環境なしで表示を切り替えられるようにする。

## HTML

追加 Python package を必須にせず、CSS/JavaScript/SVG を埋め込んだ自己完結 HTML を
生成する。元 bag を持たない利用者もレポートを閲覧できるようにする。

## 互換性

- runtime node、topic/service 契約、提出 interface は変更しない。
- `multi_purpose_mpc_ros` の install へ package と CLI wrapper を追加する。
- source checkout から `python3 -m localization_scope` でも実行できるようにする。
