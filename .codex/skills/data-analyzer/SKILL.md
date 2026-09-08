---
name: data-analyzer
description: rosbag/MCAPのtopic頻度・欠損・時刻と制御/軌跡/自己位置の因果関係を解析する。結果JSONだけの集計はevaluation-analyzer。
---

# Run time series

run ID、Domain、区間、commit/configを固定する。`output/latest/`を使う場合は
log/JSON/MCAPの各リンク先が同じrunか確認する。
ROS依存の読取はコンテナ内の`ros2 bag info`等を使い、読めないデータは未解析と明記する。

- receive time、source stamp、/clock、steady clockを区別し、単位と時間原点を合わせる。
- `/control/command/control_cmd`、`/localization/kinematic_state`、
  `/planning/scenario_planning/trajectory`と原因候補のsensor/V2Xを必要な区間で比較。
- topic頻度・gap、速度/操舵/加減速、NaN/Inf、停止・振動、pose jump、frame不一致を調べる。
- bagの記録欠損と実際のpublish停止、throttleログと全周期の観測を区別する。

指標は区間・sample数・時間系とともに示し、異常の最初の境界と反証できる原因候補を報告する。
元bagは保存し、抽出データや図をsourceへコミットしない。
