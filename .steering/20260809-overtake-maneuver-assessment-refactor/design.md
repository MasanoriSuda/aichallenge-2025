# Design

## Existing coupling

`mpc_controller_cpp.cpp`内で、左右の`SideAssessment`から以下を個別に計算している。

- Mission物理余裕
- current/alternateの実行可能性
- alternateを評価すべきか
- pending sideとpending開始時刻
- stable時間

次段でrear-clear時間、最低速度、Return成立性を追加すると、この局所状態更新へ評価ロジックが
さらに混ざるため、純粋なcoreへ抽出する。

## New core boundary

1. `assess_pass_maneuver_candidate`
   - controller固有の`SideAssessment`を、共通のPass候補へ射影する。
   - Mission由来の物理余裕、rear-clear予測時間、最低速度を保持する。
2. `compare_opponent_side_maneuvers`
   - 現行と反対側のfeasible/reserveを比較する。
   - 現行と同一の「current不成立、または余裕差が閾値以上」を返す。
3. `update_side_replan_debounce`
   - pending side、開始時刻、連続stable時間を更新する。
   - 今回は現行と同じ連続安定契約を維持する。

既存`resolve_opponent_side_replan`はno-return、target continuity、replacement countなどの
戦術ポリシー所有者として残す。

## Compatibility

- 候補生成と安全ゲートは変更しない。
- 数値比較式と閾値は変更しない。
- 新しいcore関数はROS型に依存しない。

