# Design

## 現行課題

`output/20260813-234438`では、MPCC-lite shadowが反対側をbestと判定しても`authority=none`となる場面がある。MPCC-liteの勝者が、別系統の`opponent_side_replan_ready`を再度満たさなければ実行権限を得られないためである。

また、最初のcross-side置換時に`mission_cross_side_transition_committed`とno-returnを即座にlatchするため、targetがまだ十分前方でも2回目の再選択が不可能になる。

## 統合する切替契約

MPCC-liteの反対側勝者について、次を満たす場合だけcross-side候補としてdebounceする。

1. ShiftOutまたはPass中である。
2. target観測が連続し、現在車体が非重複で予測が有効である。
3. commit stageが`ShiftCommitted`で、side-by-side/no-return前である。
4. replacement回数上限内である。
5. 勝者がrear-clear/Returnまで成立した完全Missionである。
6. 現在側holdよりMPCCスコアが設定値以上良い、または現在側が不成立である。
7. 同じ反対側が設定時間だけ連続して勝つ。

成立時はMPCC-liteが旧side comparatorを介さず、既存のtransactional cross-side preflight/commitへ直接完全Missionを渡す。

## no-return

`lateral_clearance_latched`だけではside-by-sideとは扱わない。targetが設定距離内へ入る、forward completionがlatchする、rear-clearになる、SafeSeparationへ入る、のいずれかでno-returnを固定する。

MPCCによる計画上のside変更そのものはno-returnにしない。これによりtargetが前方にいる間は左右を再評価できる。実際のside-by-side到達後は従来どおり単調latchし、全幅切替を禁止する。

## チャタリング対策

- tactical評価周期: 0.15 s。
- 同一反対側の安定時間: 既存`opponent_side_replan_stable_sec`（0.10 s）。
- 最小スコア差: 既存`opponent_side_replan_min_progress_score_advantage`（0.35）。
- Mission内cross-side上限: 3回。
- 置換後はpending候補を消去し、次の候補は改めて安定確認する。

## 効果確認

- `MPCC-lite shadow best=<opposite>`が持続した場合に`authority=replace`となること。
- `OvertakeLine opponent side PassPlan replaced`がno-return前に発生すること。
- side-by-side後にcross-side置換が発生しないこと。
- `Pass -> Return -> Idle`完遂率、SafetyBrake、Recovery、壁接触を比較すること。

