# Design

## 設計判断

知覚と安全監視は毎周期更新されている一方、戦術は mission 開始時の frozen plan を中心に動作している。これを、実行計画と shadow 候補評価を分離した限定的な receding-horizon 戦術へ変更する。

実行中の経路は引き続き固定する。左右両候補を定期評価し、明確な優位と安全性が確認された場合だけ、完全な `OvertakePassPlan` を原子的に置換する。

```text
current frozen plan ────────────────> execute unchanged
        │
        └─ periodic shadow evaluation
             ├─ current still valid / advantage small -> keep
             ├─ alternate valid and clearly better   -> atomic replace once
             └─ neither valid                         -> SafeSeparation / Follow
```

## 適用フェーズ

shadow 評価の対象は次に限定する。

- 同一 target を保持した ShiftOut
- target が十分前方に残る初期 Pass
- SafeSeparation/FollowPrepare へ移る直前の救済判定

Return と Recovery では実施しない。FollowPrepare 中に評価する場合も、現在の lateral recovery を直接反転させず、安全な新規計画を現在状態から生成できる場合だけ再開する。

## No-return point

次のいずれかを満たした時点で side switching を禁止する。

- target 前方距離が `no_return_min_front_distance` 未満
- current body footprint が target と重複
- target との横並び開始を示す front-overlap/lateral-clear 状態
- rear-clear が成立
- Return または Recovery に移行済み
- side replan を既に 1 回実行済み
- target ID/位置/course progress が不連続

初期値は target 前方距離 3.0～4.0 m の範囲で開始し、実走ログから決定する。既存の `target_intrusion_guard_distance=2.0 m` より手前で切替を閉じ、横並び直前の横断を防ぐ。

## Shadow candidate

現在状態から左右それぞれについて、既存の mission candidate lattice と静的・動的 preflight を再利用して完全経路を作る。

評価対象は以下である。

- 現在位置から候補 goal までの横遷移
- rear-clear までの Pass continuation
- Return corridor
- 全経路の最小物理壁余裕
- target および他車両との現在・予測 footprint separation
- swept footprint corridor
- steering curvature と横加速度
- rear-clear 予測時間と prediction slack
- mission の残り絶対時間・距離 budget

相手の瞬間横位置だけを比較せず、完全経路が成立する candidate のみを優位判定へ渡す。

## 切替判定

pure core に、現在側と反対側の評価結果を入力して決定だけを返す resolver を追加する。

候補アクションは次とする。

- `KeepCurrent`
- `ReplaceWithAlternate`
- `FallbackSameSide`
- `BlockedByNoReturn`
- `WaitForStability`

切替は次のどちらかで許可する。

1. 現在側が不成立、反対側が成立し、その状態が短時間継続した。
2. 両側成立だが、反対側の最小物理余裕が現在側より 0.3～0.4 m 以上大きい状態が 0.25 秒継続した。

初期設定案:

- shadow evaluation interval: 0.10～0.20 s
- minimum physical-reserve advantage: 0.35 m
- stable time: 0.25 s
- maximum replacements per mission: 1
- no-return minimum target-front distance: 3.5 m

現在側の hard failure 時も単発 V2X ノイズによる反転を避ける。ただし壁・実 footprint 衝突など即時安全処理は従来どおり shadow 判定より優先する。

## Atomic replacement

`ReplaceWithAlternate` が確定した場合、以下を一括更新する。

- `mission_plan`
- `mission_path_frozen`
- `pass_side_sign` と behavior side lock
- fixed corridor goal
- ShiftOut/Pass/Return distance
- closing-speed policy
- horizon/transition plan
- side-replan candidate debounce state
- lateral-clear/front-cap latch のうち新経路で再検証が必要な状態
- mission generation と side replacement count

置換途中の旧 plan と新 plan を混在させない。Pass 全体の開始時刻、累積距離、絶対 budget、target lock は維持する。

## Fallback ordering

現在側が継続できない場合の処理順を次へ変更する。

1. 現在側の完全経路を再検証
2. no-return 前なら反対側の完全経路を shadow 評価
3. 反対側成立なら atomic replacement
4. 不成立なら same-side SafeSeparation
5. SafeSeparation も不成立なら FollowPrepare または Recovery

物理接触、wall contact、emergency risk、solver recovery request はこの順序より優先する。

## Target lateral trend

相手横速度は、初回実装では切替の必須条件にしない。まず現在・予測 footprint と完全候補の物理余裕で成立させる。

将来追加する場合は、course-frame lateral velocity を 0.3～0.5 秒程度平滑化し、candidate score の小さな補正に限定する。横速度だけで計画を切り替えない。既存設定を単純に `true` にする A/B とは分離する。

## ログ

過剰な毎周期ログを避け、状態変化時だけ以下を出す。

- shadow 評価開始/終了
- current/alternate side と成立可否
- 各候補の最小物理余裕、rear-clear 予測、主要 reject reason
- stable timer の成立
- no-return へ入った理由
- atomic replacement の旧側、新側、target、位置、mission generation
- 反対側成立中に SafeSeparation/FollowPrepare が選ばれた場合の invariant violation

周期 debug には集約値だけを追加する。

- `opp_replan_eligible`
- `opp_alt_side`
- `opp_alt_feasible`
- `opp_advantage`
- `opp_stable`
- `opp_no_return`
- `opp_switch_count`

## テスト

pure core で以下を網羅する。

- 現在側成立、反対側優位不足なら保持
- 現在側不成立、反対側成立なら安定確認後に置換
- 一時的な反対側成立では置換しない
- 両側不成立なら same-side fallback
- target が近い、重複中、横並び、Return/Recovery なら no-return
- 2 回目の切替を拒否
- target discontinuity と stale V2X を拒否
- 絶対 Pass budget を置換でリセットしない
- 置換後の frozen plan が一貫して同じ generation を使用する

controller 側では、通常の frozen mission 中にも shadow 評価が到達可能であることと、atomic replacement 後に旧側状態が残らないことを確認する。

## 動的評価

`make dev2` で、先行車が現在側へ寄り反対側が開く区間を対象とする。

合格条件:

- 反対側が 0.25 秒以上成立した機会をログで検出できる。
- no-return 前なら 0.5 秒以内を目安に新 plan へ置換する。
- 1 mission の切替は最大 1 回。
- 横並び後の切替は 0 回。
- 反対側成立中の直接 FollowPrepare/SafeSeparation 遷移は 0 回。
- 置換後に rear-clear と Return まで完遂する。
- wall Recovery、接触、SafetyBrake、solver failure を現行基準より増やさない。

追い越し成功率と同時に、好機検出数、置換数、no-return 抑止数、誤切替数を集計する。

