# Design

## 1. Same-side prefix authority

`resolve_mpcc_lite_authority()`の同側分岐を、完全Missionだけでなく
`selected_prefix_execution_admitted`にも対応させる。

prefix自体の安全性は既存の`resolve_mpcc_lite_prefix_execution()`で再確認し、commitは
既存の`replace_frozen_overtake_mission_after_dynamic_replan()`を使用する。したがって
shadow scoreだけで経路を書き換えない。

## 2. 置換クールダウン時計

`mission_planner_generated_at_sec`はV2X予測epoch/TTLの管理用であり、active Missionの
予測更新でも変化する。置換チャタリング防止には使わない。

`mission_last_tactical_replacement_sec`を追加し、Mission freeze/transactional replacement
の成功時だけ更新する。同側置換の最小間隔はこの時刻から計算する。

## 3. Full-track transition contract

完全Mission admissionでは`transition_required`だけでなく、実際に計算した
`outer_transition_preflight.feasible`を`is_full_track_transition_admitted()`へ渡す。
これにより将来の全幅切替が必要なのにpreflightされていないMissionを凍結しない。

## 4. 非対象

同期MPCC-lite評価のlatest-only worker化、Recovery別プロセス化、パラメータ攻撃化は
別ステアリングで扱う。
