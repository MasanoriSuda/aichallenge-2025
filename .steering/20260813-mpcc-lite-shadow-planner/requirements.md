# Requirements

## 背景

`a20fa0b` の速度連成により、末尾の妨害区間を除いた試走では追い越し完遂が
前回の 2/10 から 4/8 へ改善した。一方で SafeSeparation 進入は11回、
壁clamp・横加速度Recoveryは3回残った。

現行は左右候補、現在側継続、Returnを別々のFSM判断で扱うため、それぞれの
rear-clear時間・最低速度・壁/車両余裕を同じ尺度で継続比較できていない。

## 目的

1. left / right / current-side hold / Returnを同一の有限horizon指標で評価する。
2. 現FSMが採用した戦術とshadow推奨戦術の差を走行ログへ残す。
3. 次段階でPass実行層へ採用できるか、制御を変えずに予測妥当性を確認する。

## 制約

- Phase 1では制御出力、FSM遷移、side lock、速度上限を変更しない。
- 既存の左右Mission candidateとreceding-horizon結果を再利用する。
- 壁、車両、横加速度、rear-clearの未検証候補はfail-closedで評価する。
- 8 Hzでshadow評価し、通常ログは1 Hz以下に抑える。
- 新解が得られない周期はlast-feasible診断を保持するが、制御には使わない。
- ROS 2 topic/service、評価schema、`aichallenge_system`は変更しない。
- `aichallenge/result-summary.json`のユーザー変更は含めない。
