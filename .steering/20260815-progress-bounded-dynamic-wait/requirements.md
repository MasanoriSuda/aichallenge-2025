# Requirements

## Background

`output/20260815-203758` のepisode 2では、Pass entry physical wall gate不成立後に
DynamicMissionWaitへ入った。設定上の再選択leaseは0.75秒／4 mだが、forward prefixが
activeという理由だけでrear-clear待ちが継続し、約9.37秒後に失効Missionへ復帰した。
その間、target longitudinalは約7.7 mから19.5 mへ悪化していた。

## Requirements

- 非terminal DynamicMissionWaitの短い再選択leaseを通常は0.75秒／4 mで守る。
- lease超過後の継続は、物理的に有効なforward prefixだけでなく、全closing権限または
  runtime検証済みcontinuous DP authorityを要求する。
- lease超過後の継続は、相手に対する前進進捗が直近lease時間内に確認できる場合だけ許可する。
- 条件を失った場合はRecoveryへ落とさず、既存の失敗側blockとfresh左右探索へ戻す。
- target discontinuity、実wall contact、body overlapなどのhard faultは従来どおりfail closedとする。
- terminal Pass budget修正とROS interface、設定値は変更しない。

## Constraints

- `aichallenge/result-summary.json` のユーザー変更は変更・コミットしない。
- recoverable side contactと有効なcontinuous DPの短い継続を壊さない。
- requestのboolean aggregate初期化を増やさず、明示field assignmentへ整理する。
