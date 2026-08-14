# Design

## 実走で確認した不足

`output/20260814-231327/d1/autoware.log` ではDP評価は186回動作し、feasibleな候補も
選択されていた。しかし候補ログの `dp=checked/feasible/cost/bridge` は全て
`bridge=0`であり、実行側は単一横goalのままだった。

結果は追い越し開始10回、Pass到達8回、rear-clear 2回、正常Return完遂1回で、主な
失敗は `physical target separation conflicts with wall bounds` と壁制約だった。

## データ経路

1. side assessmentが作った `FrenetDpCorridorBranchResolution` を、選択対象の
   `OvertakeMissionCandidate`へコピーする。
2. `freeze_selected_overtake_mission` が候補を検証し、active Mission stateへ距離列・
   横位置列をatomicに保存する。
3. active Mission中の前進距離をphase遷移とは独立に積算する。
4. ShiftOut/Passの各control cycleで、
   `execution_distance = mission_traveled + horizon_distance` としてDP列を補間する。
5. 補間列を `evaluate_overtake_line_horizon` のlateral overrideへ渡す。
6. 既存receding-horizon optimizerがその列を参照に、最新の壁・相手hard bounds内で
   最適化する。

## 有効条件

DP実行参照は次を全て満たす場合だけ有効とする。

- config enabled
- phaseがShiftOutまたはPass
- active MissionのsideとDP sideが一致
- 距離列と横位置列が同じ長さで2点以上
- 全要素がfinite、距離が非負かつstrict increasing
- 現在horizonにDPで覆える有効prefixが2点以上ある

coverageがhorizon途中で切れた場合は覆えるprefixだけDP参照を使い、未被覆tailは従来
goal参照を維持する。有効prefixが2点未満になったらDP終端横位置を無期限保持せず、
参照列全体を従来goalへ戻す。現在の動的rolling replanとlast-feasible leaseが次の
Missionを供給する責務を維持する。

## 進行距離

DP列は候補生成時のegoを0 mとする。Mission stateにphase非依存の
`mission_frenet_dp_execution_traveled_m`を持たせ、既存の観測gap上限を使って前進距離を
積算する。Mission replacement時は新しい候補が現在ego基準なので0へ戻す。

FollowPrepareではDP参照をpublishしないが、車両が前進した分は積算し、resume時に古い
prefixの先頭へ戻らないようにする。

## 安全境界

- DP列はbaseline referenceであり、execution boundsではない。
- static map footprint、wall clearance、target separation、横加速度の既存検証を必ず通す。
- optimizerが不成立なら既存のlast-feasible/dynamic wait/Recovery処理へ委譲する。
- Return開始時はDP参照を解除し、検証済みReturn profileを使う。
