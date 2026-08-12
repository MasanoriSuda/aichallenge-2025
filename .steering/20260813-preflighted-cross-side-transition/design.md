# Design

## 方針

`OvertakeMissionCandidate` に `outer_transition_preflight_validated` を追加し、候補生成時に scheduled outer transition の完全 preflight 成功を記録する。

Cross-side Admission は次のように判定する。

1. 追加遷移なし: 従来どおり後続の安全・budget 判定へ進む。
2. 追加遷移あり、preflight 未完了: `additional_side_transition_required` で棄却する。
3. 追加遷移あり、preflight 完了: 壁、最低速度、rear-clear、残時間・残距離をすべて再確認してから許可する。

## 安全境界

- boolean を設定するのは、既存の `outer_transition_preflight.feasible` を通過した完全 Mission だけとする。
- `build_overtake_pass_plan` でも未検証遷移を拒否し、候補生成後の取り違えを防ぐ。
- Candidate selector の数値検証にも provenance 条件を加える。
- 動的切り返し回数、no-return latch、0.25 s 安定確認は変更しない。

## 効果確認

次回走行ログで以下を確認する。

- `additional_side_transition_required` の反復が解消すること。
- `OvertakeLine opponent side PassPlan replaced` が必要時に発生すること。
- 切り返し後に scheduled outer transition、rear-clear、Return が完遂すること。
- wall / lateral acceleration Recovery と接触が増えないこと。

