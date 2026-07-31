# 設計

## 現行の問題

`update_overtake_line()` 内で次の判断と副作用が連続している。

1. execution horizon が無制約かの判定
2. constrained horizon での初回 release 許可
3. 既存 release の committed Pass hold 許可
4. front cap の release / reapply
5. 状態変化理由の選択とログ出力
6. committed Pass 速度 floor の適用

この構造では、実走で多発した front cap 再適用を直す際に、条件の優先順位、状態更新、
ログ、副作用を同時に変更する危険がある。

## 変更方針

`v2x_overtake_core` に `CommittedPassPolicyRequest` と
`CommittedPassPolicyResolution` を追加する。

純粋関数 `resolve_committed_pass_policy()` は入力された現在の事実から次を返す。

- active Overtake execution か
- constrained horizon release が許可されるか
- committed Pass speed hold が許可されるか
- front cap を解除状態にするか
- front cap 状態を更新・ログ出力すべきか
- speed hold / constrained horizon release が現在 active か
- committed Pass 速度 floor を使うか
- front cap 状態変化の理由

既存の `can_release_overtake_front_cap()` と
`should_apply_committed_pass_speed_floor()` は下位の判断関数として維持し、
新しいポリシーから呼び出す。

controller は horizon、車両相対位置、設定値などの事実を request へ詰め、
resolution を出力と内部状態へ適用するだけにする。

## 診断理由

理由は enum と `to_string()` で表現し、現行文字列を維持する。

- physical lateral clearance; constrained feasible Pass horizon accepted
- lateral goal and execution horizon clear
- lateral goal complete and locked target no longer ahead
- locked target unavailable
- lateral goal incomplete
- execution horizon constrained
- lateral clearance below reapply threshold

## 互換性

- `/control/command/control_cmd` を含む topic 契約は変更しない。
- Domain、V2X message、launch、config schema は変更しない。
- `aichallenge_system` は変更しない。
- 速度値・操舵値・状態遷移は変更しない。

