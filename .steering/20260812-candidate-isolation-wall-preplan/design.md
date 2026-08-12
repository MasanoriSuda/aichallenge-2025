# Design

## 1. Candidate-local fault isolation

`select_overtake_mission_candidate`の`valid`をrequestレベルの妥当性として扱う。

- request設定異常: `valid=false`
- 個別candidate数値異常: そのcandidateだけskip
- 有効candidateなし: `valid=true, found=false`
- 有効candidateあり: 元vectorのindexを保持して選択

棄却数をselection metadataへ保持し、既存のside diagnosticへ出す。

## 2. Runtime wall preplan

実車体footprintを次の2段階でsamplingする。

1. hard footprint = vehicle footprint + `min_wall_clearance`（現行0.15 m）
2. warning footprint = hard footprint + `runtime_wall_preplan_reserve`（0.10 m）

hard footprintが接触した場合は従来どおりRecoveryとする。warning footprintだけが接触し、
current body分離、target continuity、prediction、fresh same-side complete Missionが成立する
場合は、既存transactional replacementを使って現在位置起点のMissionへ置換する。

fresh candidateがまだない場合は`opponent_side_replan_last_evaluation_sec`を失効させ、
次回planner評価を即時要求する。古いMissionや反対側候補を予告帯だけで採用しない。

置換はMissionあたり回数とcooldownを持ち、警告帯境界での再置換チャタリングを防ぐ。

## 3. Rejected alternate retry throttle

cross-side replacementがadmissionで棄却された場合、次を記録する。

- candidate side
- candidate lateral goal
- rejection時刻

同じsideかつgoal差が`max_target_change`以内なら0.25秒はcommitを再試行しない。候補の
read-only評価は維持する。横goalが実質的に
変化した場合は外部状態が変化したものとして即時再評価を許可する。成功時とMission reset時に
cacheを破棄する。

## 4. Fail-closed境界

次はpreplanで上書きしない。

- physical wall contact
- hard wall margin violation
- wall sample unavailable/out-of-map
- target discontinuity / position jump
- current bodyの非recoverable overlap
- solver Recovery

## 5. ログ

状態変化時だけ次を出す。

- candidate selectorで局所棄却した候補数（既存side reasonへ付加）
- runtime wall preplan warning/request
- runtime wall same-side Mission replacement
- runtime wall replacement rejection

warningが継続する40 Hz全周期では出力しない。
