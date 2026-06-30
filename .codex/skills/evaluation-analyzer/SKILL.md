---
name: evaluation-analyzer
description: Automotive AI Challenge の評価結果解析スペシャリスト。`output/latest`、`result-summary.json`、`dN-result-details.json`、penalty、lap time、finish/timeout、AWSIM/Autoware ログを読み、評価失敗やスコア低下の原因候補を整理する。「評価結果を解析」「make eval の結果を見て」「ペナルティ原因を調べて」などで呼び出される。
---

# Evaluation Analyzer

評価ランの結果を読み、完走状況、ラップ、ペナルティ、タイムアウト、ログ上の異常を整理する。

## 読む順序

1. `output/latest/d<N>/result-summary.json`
2. `output/latest/d<N>/result-details.json`
3. `output/latest/d<N>/autoware.log`
4. `output/latest/docker_build.log` または `output/latest/docker_run.log`
5. 必要なら `output/<run_id>/` 配下の実体ファイル

`latest/` のリンクが壊れている場合は、最新の `output/<YYYYMMDD-HHMMSS>/` を探す。

## 確認項目

- `schema_version`
- `finished`
- `lap_count` / `required_laps`
- `min_lap_time` / `avg_lap_time` / `total_lap_time`
- `penalty_count`
- `penalty_total_seconds`
- `penalty_events[].kind`
- `session_timeout`
- `vehicles[].final_position`

## 解釈

- `finished=false`: 制御出力、自己位置、trajectory、engage、AWSIM state のどこで止まったかを追う。
- lap が 0: initial pose、control engage、trajectory、control command を優先確認する。
- penalty 多発: 壁接触、コース外、衝突、速度過大、軌跡追従不良を疑う。
- result JSON 欠落: AWSIM 起動、評価 FSM、output path、プロセス終了タイミングを確認する。

## 出力

```markdown
# Evaluation Analysis

## Result
- 完走/未完走、ラップ、タイム、ペナルティ

## Evidence
- JSON とログの根拠

## Likely Causes
- 可能性順の原因候補

## Next Actions
- 確認コマンド
- 修正候補
```

保存が必要な場合は `.log/evaluation/YYYYMMDD-aic-evaluation.md` に書く。
