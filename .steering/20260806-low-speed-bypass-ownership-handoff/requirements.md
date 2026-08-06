# Requirements

## 目的

通常の `OvertakeLine` と停止車用 `LowSpeedAvoidance` が同じ対象の横経路を同時に所有し、成立済み追い越しを破棄した後に停止・証拠なし Reverse へ連鎖する問題を解消する。

## 確認済み事象

`output/latest/d1/autoware.log` では次の順序を確認した。

- `OvertakeLine: Idle -> ShiftOut`
- `V2X behavior: Overtake -> LowSpeedAvoidance`
- `OvertakeLine: ShiftOut -> Idle, reason=stopped vehicle bypass owns target`
- `Low-speed direct control stopped by live safety guard: live vehicle corridor unavailable`
- `LowSpeedAvoidance -> Follow, front_distance=inf`
- `evidence_free_qualified=1`
- `SHIFT_TO_REVERSE`, `wall=none`, `v2x_blocker`なし

同じrunで低速direct controlのcorridor停止は4回、証拠なしStuck確認は3回発生した。

## 必須動作

- 凍結済みの同一target `ShiftOut` / `Pass` は、targetが停止判定へ変化しただけでは `LowSpeedAvoidance` に横取りさせない。
- commit済み通常追い越しでは既存の opponent-driven side replan、Pass horizon、SafetyBrakeを継続利用する。
- 停止車用direct controlを新規に開始したケースでは、固定側corridorが不成立になった場合に反対側を現在poseから再検証する。
- 反対側はV2X corridorとstatic wall swept-footprintの双方が成立した場合だけ採用する。
- active中に側を切り替えた場合はPass速度を維持せず、Shift速度へ戻して横断する。
- 両側不成立時は従来どおり停止し、未検証経路へ突入しない。
- ログは状態変化時だけ出し、通常周期ログを増やさない。

## 非目標

- gap、壁余裕、車体寸法、加速度上限の緩和。
- stuck recovery全体の再設計。
- solver fallback経路の変更。
- ROS 2 topic、message、launch、評価インターフェースの変更。

## 完了条件

- commit済みShiftOut/Pass中に同一targetの停止車判定が発生しても、通常追い越しの所有権が保持される。
- activeな低速回避の現在側が不成立、反対側が成立の場合、反対側へ切り替わる。
- active側切替時にdirect control phaseが`Shift`へ戻る。
- 既存の低速回避、OvertakeLine、stuck recovery単体テストが成功する。
- `make autoware-build`が成功する。

