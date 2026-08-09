# Design

## 1. Commit段階

既存の多数のbool latchを次の4段階へ射影する純粋関数を追加する。

- `Selectable`: frozen Missionなし。左右を通常選択できる。
- `ShiftCommitted`: frozen Missionあり、まだ横並びcommit前。完全な代替Missionへの置換を許す。
- `SideBySideCommitted`: lateral-clear/forward-completion latch、またはno-return距離内。sideを固定する。
- `RearClear`: rear-clear成立。Returnを優先する。

FSM自体はこの変更で全面置換しない。まず既存のopponent-side replanの許可条件を
この段階判定へ集約する。

## 2. 予測横侵入の早期検出

現在位置ではorderingが成立していても、予測位置がordering margin近傍まで
選択側へ移動している場合をearly intrusion riskとする。

- 現在と予測のtarget-relative lateralが有限。
- 予測値が選択側orderingを悪化させている。
- 予測orderingが `2 * ordering_margin` の警戒帯へ入る。
- `ShiftCommitted`中のみ有効。

このriskがある場合、通常の0.15秒周期より早く候補評価を許し、現在Missionの
runtime sweepを不成立として比較する。代替Missionの完全preflightと0.25秒の
安定待ちは維持するため、単発ノイズだけでは左右切替しない。

## 3. no-return

`opponent_side_replan_no_return_front_distance` を3.5 mから2.0 mへ変更する。
これは危険経路を直接許す変更ではなく、完全preflight済み代替Missionを検討できる
時間窓を約1.5 m延ばす変更である。2.0 m内では`SideBySideCommitted`としてside固定する。

## 4. Direct Pass handoff

base racing lineが既にclearなため `Idle -> Pass` へ直接入る周期では、Behavior層の
locked-target出力が旧Idle状態のままになる。選択済みtarget/Missionは既にfreeze済みなので、
その1周期はline出力を開始せず通常MPCへ譲り、次周期のlocked-target観測からPassを実行する。

壁接触等のentry hard guardはこの判定より前に従来どおり評価される。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: commit段階、早期侵入、handoff判定の純粋関数。
- `mpc_controller_cpp.cpp`: Behavior評価とDirect Pass遷移へ統合。
- `config/config.yaml`: opponent-side no-returnを2.0 mへ変更。
- `test_v2x_overtake_core.cpp`: 境界条件テスト。
