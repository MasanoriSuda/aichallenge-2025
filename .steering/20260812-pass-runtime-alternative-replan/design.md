# Design

## 現行不整合

現行のopponent-side assessmentはFollowPrepare中も現在側を再評価できる。一方、
dynamic Mission wait admissionは`before_no_return && replacement_count_available`を
必須としている。このため、横並び後に現在側の将来経路だけが悪化したケースでも
waitへ入れず、TacticalRevalidationから通常FollowPrepareへ移ってMissionを失う。

さらに、wait中のcurrent maneuver評価が失敗済みMissionのpredicted sweepを参照し、
fresh same-side candidateが完全preflight済みでもcurrent candidateを棄却し得る。

## 変更方針

### 1. wait admissionを置換方向別にする

DynamicMissionWait admissionへsame-side replacement permissionを加える。

- same-side: no-return後も許可
- cross-side: `before_no_return && replacement_count_available`のときだけ許可

current body非重複、target continuity、prediction、wall/solver hard faultの既存条件は
共通で維持する。

### 2. paused fresh candidateをcandidate自身で評価する

dynamic wait中に生成したcurrent-side candidateは、完全Mission preflightを通った
candidate自身を評価対象とする。失敗した旧Missionのpredicted overlapは、新世代の
候補を棄却する入力にしない。

### 3. atomic replacement

fresh current Missionを`replace_frozen_overtake_mission_after_dynamic_replan`へ渡し、
committed same-side replacementを明示的に許可する。置換は新PassPlanの生成成功後だけ
commitし、失敗時はrollbackする既存処理を使う。

FollowPrepareからの置換は現在位置を起点にShiftOutを再開する。Pass経過時間、距離、
extension count、Mission総時間は既存どおり引き継ぐ。

### 4. cross-side制約

dynamic wait解決時に`alternate_replacement_allowed`を渡す。no-return latch後はalternate
candidateが存在しても採用せずsame-sideを待つ。これにより横並び中の全幅横断を防ぐ。

## hard fault

次は従来どおり即Recovery対象とする。

- actual wall contact / wall margin violation / wall sample unavailable
- current body overlap
- target discontinuity / position jump
- solver recovery
- overtake forbidden hard state

fresh complete candidateが存在しcurrent bodyが分離している場合だけ、FollowPrepare中の
縦距離由来Emergencyを置換処理より後順位にする。

## ログ

既存ログを利用する。

- `dynamic mission wait entered`
- `fresh same-side PassPlan replaced`
- `dynamic Mission wait failed`
- `Mission generation invalidated`

ROS topic/service、評価schema、速度・壁余裕パラメータは変更しない。
