---
name: evaluation-analyzer
description: AWSIM評価の完走・lap・penalty・timeoutと結果JSON欠落を調べる。rosbag時系列が必要ならdata-analyzerへ進む。
---

# Evaluation results

`output/latest/`は探索の入口。リンク先のrun ID、Domain、commit/configを固定し、
同じrunの`result-summary.json`、`dN-result-details.json`、`autoware.log`を照合する。
リンク欠落時も単に最新時刻のrunを採用せず、対象との対応を確認する。

- schema_versionを確認し、実在するキーからfinished、lap_count/required_laps、
  lap time、順位、penalty件数/秒数/イベント、timeoutを集計する。
- lap 0や未完走はinitial pose→engage→localization→planning→control→AWSIM stateを追う。
- penaltyは発生時刻と車両挙動へ結び付ける。JSON欠落を制御失敗や合格と同一視しない。

結果、根拠ファイルと時刻、原因候補、未確認項目を示す。
警告行数を事故件数に数えず、複数runや再試行は分けて報告する。解析目的で結果を書き換えない。
