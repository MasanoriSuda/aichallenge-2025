# Requirements

## Background

`output/20260815-200955` では追い越しepisode 1がPass距離40 mを超えた後、
`Pass -> FollowPrepare -> Pass` を短時間に14回繰り返した。失効済みMissionの
current-side候補が通常のDynamicMissionWait復帰条件だけで再採用され、直後に同じ
absolute Pass distance limitへ再到達していた。

## Requirements

- absolute Pass time/distance limitをterminal budget abortとして区別する。
- terminal abort後の同側復帰は、現在状態から再検証したprogressive prefixが
  target/body prediction、wall、minimum speed、短いtime/distance budgetを全て
  満たす場合だけ、1 episodeにつき1回だけ許可する。
- terminal abort後も、成立済みの反対側complete Missionは従来どおり優先する。
- どちらも成立しない場合、1回の左右評価完了後に失敗側を短時間blockし、
  Recoveryや15秒待機を挟まずfresh searchへ戻す。
- hard fault、body overlap、target discontinuityは従来どおりfail closedとする。
- ROS topic/service、設定値、評価interfaceは変更しない。

## Constraints

- `aichallenge/result-summary.json` のユーザー変更は変更・コミットしない。
- terminal以外のDynamicMissionWait挙動は変更しない。
- requestのboolean aggregate初期化を増やさず、明示field assignmentへ整理する。
