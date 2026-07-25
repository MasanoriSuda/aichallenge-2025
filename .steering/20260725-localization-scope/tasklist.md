# Localization Scope タスクリスト

## 文書

- [x] requirements.md
- [x] design.md
- [x] README / README.ja.md

## Metadata / repository

- [x] metadata schema と example
- [x] metadata template (`init`)
- [x] repository/default path discovery
- [x] config/trajectory hash と manifest

## Analysis

- [x] trajectory CSV loader
- [x] trajectory projection
- [x] rosbag reader と topic normalization
- [x] topic availability/rate
- [x] single-run metrics
- [x] speed-band metrics
- [x] two-run comparison/classification

## Output

- [x] summary.json / run-manifest.json
- [x] report.html
- [x] comparison-summary.json / comparison.html
- [x] runs-index.json / browser catalog.html

## Integration

- [x] `python3 -m localization_scope`
- [x] `ros2 run multi_purpose_mpc_ros localization_scope`
- [x] CMake/package dependencies

## Verification

- [x] metadata/trajectory/analysis tests
- [x] synthetic single report
- [x] synthetic two-run comparison
- [x] current `output/latest/d1` MCAP report
- [x] build or targeted syntax/import verification

## 検証結果

- `ament_flake8`: 11 files、問題なし
- host pytest: 5 passed
- `make autoware-build` 相当の Compose build: 25 packages successful
- targeted colcon test: `test_localization_scope` 5 passed
- `ros2 run multi_purpose_mpc_ros localization_scope --help`: 成功
- 最新 d1 MCAP の単一 report: 成功
  - 現行 bag から EKF cross-track P95 を算出
  - 未記録の GNSS/IMU topic は warning / N/A
- 最新 d1/d2 MCAP の 2 run comparison: 成功
  - 取得可能 metric を判定
  - GNSS/IMU依存 metric は判定不能
- 最新 d1/d2 MCAP の browser catalog: 成功
  - 2 run / ordered comparison 2件を生成
  - Single / Baseline vs Candidate selectorをHTMLへ埋め込み
- catalog埋め込みJavaScript: Node.js構文検証成功

`colcon test-result --verbose` は全体集計時に既存
`build/joycon_contract_guard/package.xml` 不在の診断を出したが、集計結果は
624 tests / 0 errors / 0 failuresであり、追加testは成功した。

## Deferred

- 3 run 以上の trend comparison
- runtime MPC 内部 reference trajectory debug topic
- EKF innovation/NIS（現行 bag に内部値がない）
- simulator ground truth comparison
