# Design

## 確認した構造不具合

最新走行では同一attemptが有効なまま、物理壁確認済みのDynamicEscapeから
RacingLineへ実行権が戻っていた。入口用の4.5 m判定が閉じた周期に候補が消え、
exit contractが旧解を0.35秒だけ保持し、`target-blocking`をwall replan failureとして
backoffへ投入していた。

このため下流の保持処理を改善しても、上流で経路が破棄されるたびに同じ問題が再発する。

## 状態所有の整理

DynamicEscapeを次の二つの責務へ限定する。

1. Encounter lifecycle
   - 新規entry要求からattemptを開始する。
   - 同一targetが観測中、または短いtarget-loss grace中は計画要求を継続する。
   - 新規entry gateは継続可否を所有しない。
   - hard releaseだけがattemptを終了する。
2. Execution handoff
   - live candidateが消えた場合だけ、物理確認済みの保持解へ短時間引き継ぐ。
   - target-blockingは計画継続理由であり、solver failure backoff理由ではない。
   - wall rejectionやreplacement lossだけをfailure replanとして扱う。

通常Overtake Mission、Recovery、EmergencyBrakeは従来どおり上位のhard ownerとする。

## 実装方針

- attempt resolutionへ`planning_requested`と`continuation_requested`を追加する。
- GapPlannerを呼ぶ前にlifecycleを更新し、raw entry要求ではなくlifecycleの有効計画要求を使う。
- dynamic corridorの一周期欠損時も、同一のfront targetならattempt identityを保持する。
- lateral authorityは新規entryではFollowを要求するが、継続中attemptでは一時的な
  Cruise/entry gate変化だけで解除しない。
- exit contractへ`attempt_active`と`continuation_planner_requested`を渡し、継続計画中の
  `target-blocking`ではfailure replan/backoffを発行しない。
- attempt切替時はexit contractのtarget/side/branch metadataを同じ周期で更新する。

## ログ設計

- lifecycleログへraw entry、effective planning、continuation、target observationを出す。
- wall admissionの正常incomingは毎周期出力せず、hold/release/source変更時だけ出す。
- exitログは状態変化を即時、未解決状態のheartbeatを2秒周期で出す。
- `target-blocking`と`wall/solver failure`を別フィールドで識別できるようにする。

## 不変条件

- active attemptなら`planning_requested=true`である。
- entry gateの不成立だけではactive attemptを解除しない。
- target-blockingだけではfailure backoffを増加させない。
- incoming、retained、outgoingのうち実際にpublishする横制御源は一つだけである。
- retained solutionはattempt/target/side一致かつ期限内の場合だけ使用する。
