# Design

## 方針

runtime rear-clear rollout が残り絶対予算を超えたとき、Pass state に
`runtime_completion_replan_pending` を立てる。これは新しい状態機械ではなく、
次の MPCC-lite shadow 評価と実行層を結ぶ一時的な ownership latch とする。

## Shadow評価

pending 中は frozen Mission 由来の `CurrentSideHold` を候補から外す。現在側の
current-state prefix は従来どおり再生成し、安全な新解がなければ実行層が直前の
physical prefix を保持する。

次を満たす場合だけ tactical cross-side re-arm を許可する。

- Pass / SafeSeparation / pending 中
- commit stage が `ShiftCommitted`
- target が no-return front distance より前方
- target continuity、現在車体分離、予測sweep分離が成立
- execution corridor、壁、EmergencyBrake、solver の hard faultなし
- 既にcross-side transitionをcommitしていない

これによりSafeSeparation開始だけで立った履歴no-returnは再評価できるが、
side-by-side以降の全幅切替は引き続き禁止する。

## Atomic commit

MPCC-liteが fresh same-side/cross-side Missionを選んだ場合、既存の
`replace_frozen_overtake_mission_after_dynamic_replan()`を使用する。commit成功時だけ
generationを更新し、pending latchとstale shadow leaseを解除する。

## ログ

- pending開始は一度だけ警告する。
- shadow logへ `runtime_replan_pending` と `runtime_rearm` を追加する。
- 採用時は既存Mission replacementログで確認する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: tactical re-armのpure判定
- `mpc_controller_cpp.cpp`: pending lifecycle、shadow/commit接続
- `test_v2x_overtake_core.cpp`: re-arm成立・拒否条件
