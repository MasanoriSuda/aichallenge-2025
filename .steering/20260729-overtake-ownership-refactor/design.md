# 設計

## 現行の責務

| 出力 | 通常時の所有者 | Overtake 中の所有者 | 強制介入 |
|---|---|---|---|
| Behavior state | V2X behavior FSM | V2X behavior FSM | SafetyBrake |
| pass target / side | gap planner | OvertakeLine mission | early side replan、FollowPrepare 再開 |
| lateral reference | base trajectory / gap planner | explicit OvertakeLine | LowSpeed direct control、Stuck Recovery |
| velocity reference | base trajectory / Behavior | OvertakeLine stage reference | committed Pass floor |
| velocity hard limit | domain / curvature / Behavior | 上記に Recovery limit を合成 | SafetyBrake、wall contact、solver fallback |

速度は単一 owner ではなく、reference、hard limit、floor の合成である。今回この合成順序は変更しない。

## 問題

`update_overtake_line()` 内の長い `if / else if` が、そのまま遷移の優先順位になっている。
また `FollowPrepare` 再開時の side 選択は三項演算子に埋め込まれ、既存 mission side と Behavior side のどちらが勝つか分かりにくい。

## 変更方針

既存の `v2x_overtake_core` に、ROS や controller state に依存しない次の純粋関数を追加する。

### Active transition decision

入力された事実から、以下の action を現行順で一つ選ぶ。

1. physical wall contact recovery
2. ShiftOut entry wall-margin rejection
3. early Return cancellation
4. completed Pass return before wall recovery
5. completed Pass hold while return corridor is blocked
6. wall-margin recovery
7. early side switch
8. occupied-side recovery
9. rear-clear Return
10. longitudinal-progress recovery

controller は decision の action に対応する既存副作用だけを実行する。

### Execution side resolution

現行の優先順位を名前付き resolution にする。

1. `FollowPrepare` 再開かつ Behavior side が有効: Behavior side
2. 既存 mission side が有効: mission side
3. Behavior side が有効: Behavior side
4. それ以外: side なし

この方針は現行挙動を保存するためのものであり、side 固定へ変更するのは次段階とする。

## 互換性

- `/control/command/control_cmd` を含む topic 契約は変更しない。
- Domain、V2X message、launch、config schema は変更しない。
- `aichallenge_system` は変更しない。

