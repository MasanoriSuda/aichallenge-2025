# Requirements

## 背景

`20260821-234849` の試走では、DynamicEscape が主候補の将来壁リスクを3回
事前検出し、反対側候補の試算も開始した。しかし反対側候補はすべて
`pass-side gap unavailable / forced-side-empty` で棄却され、採用は0回だった。

現行 GapPlanner は `forced_pass_side_sign` を受けると、ホライズン先頭から
すべてのstageで指定側のfree intervalを要求する。現在位置が反対側へ到達する前の
stageも同じ条件で判定するため、物理的には
`現在側 -> 中央の接続区間 -> 反対側` と移れる場合でも候補を生成できない。

## 目的

1. DynamicEscapeのspeculative opposite branchだけに到達用prefixを許可する。
2. 現在のcorridorと指定側が接続するstageをgatewayとして検出する。
3. gatewayまでは現在位置と連続なcorridorを維持し、gateway以降に指定側を強制する。
4. transition期限までgatewayが現れない候補はfail closedで棄却する。
5. 主候補が即時の重大壁リスクを持ち、反対側も成立しない場合は危険な主候補を実行しない。
6. prefix、gateway、期限切れ、主候補抑止を同一decision traceで確認可能にする。

## 制約

- ROS topic、message、service、launch、提出物契約を変更しない。
- 通常GapPlanner、明示的Overtake、active Pass/no-returnの挙動を変更しない。
- 壁・車両のhard guardを緩和しない。
- `aichallenge/result-summary.json`の既存変更を触らない。
- Recoveryアルゴリズムと速度設定は変更しない。

## Definition of Done

- 反対側候補が開始点のside不一致だけで即時棄却されない。
- 接続可能なfree intervalが現れるまで連続なprefixを構成する。
- gateway以降のtargetが要求sideを指す。
- gateway不成立は専用reasonとdeadline付きでログに残る。
- unusable alternateのside=0を`invalid-side`ではなく`alternate-unusable`と分類する。
- 即時重大壁リスクの主候補をalternate不成立時に実行しない。
- package build/testが成功する。
