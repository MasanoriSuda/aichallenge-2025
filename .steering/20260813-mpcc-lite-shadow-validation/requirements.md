# Requirements

## 背景

`98cbc23` のMPCC-lite Phase 1を `output/20260813-121813` で試走した。
有効なshadow推奨103件のうちFSM一致は91件だったが、次の診断不整合が
確認された。

- left/rightが同一周期に実行可能として比較された記録が0件
- Return候補が全345件で一度もfeasibleにならない
- 実際に完遂したReturn中もshadowがhard constraintと判定する
- `hard_constraint`だけではprogressive entry、Mission予算、物理制約を区別できない
- SafeSeparationまたはMission総時間切れ直前でもholdをfeasibleと判定する例がある

## 目的

1. shadow専用評価ではleft/rightを同一周期に独立評価する。
2. 実行中Returnを、rear-clear latchの消失だけで誤棄却しない。
3. Mission総時間とSafeSeparationの残時間・残距離を候補成立性へ反映する。
4. 候補棄却理由を実験で判別できる粒度へ分解する。
5. 次段階の実制御接続前に、shadowを診断器として信頼できる状態へする。

## 制約

- shadowはside、FSM、速度、操舵、Missionを変更しない。
- shadow用SideAssessmentはretry blockなど実制御状態を変更しない。
- 既存のwall、target、横加速度、SafetyBrake、Recovery判定は緩和しない。
- ROS 2 topic/service、評価schema、`aichallenge_system`は変更しない。
- 新しい設定値は増やさず、既存のMission/SafeSeparation予算を利用する。
- `aichallenge/result-summary.json`のユーザー変更は含めない。
