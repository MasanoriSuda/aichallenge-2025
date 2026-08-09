# Requirements

## Purpose

固定済みPass Missionがno-returnへ到達する前に、現在側と反対側の完遂性能を5〜10 Hzで再比較し、相手や空きスペースの変化に応じて明確に優れた完全Missionへ一度だけ切り替えられるようにする。

## Scope

- 物理的に成立した左右Missionを次の共通指標で比較する。
  - rear-clear予測時間
  - horizon progress score
  - 最低予測速度
  - 壁・corridor・Returnの物理余裕
- 現在側が不成立なら、成立する反対側を優先する。
- 両側成立時は、性能改善が設定閾値以上で、他指標に重大な後退がない場合だけ反対側を優先する。
- 既存のno-return、0.25秒連続安定、最大1回のMission置換を維持する。
- 判定理由と各差分を周期ログへ出す。

## Non-goals

- actual overlap中やno-return後の全幅side切替は許可しない。
- 4点横軌道、複数車occupancy、接触継続、Return再最適化は変更しない。
- 壁、車体footprint、横加速度、Mission budgetのhard constraintは緩和しない。
- ROS 2 topic/service契約は変更しない。

## Acceptance

- 反対側が1秒以上早くrear-clearでき、物理余裕と最低速度を実質的に悪化させない場合、debounce後に切替要求となる。
- 反対側の物理余裕だけが大きい場合も、完遂時間と最低速度を悪化させない場合のみ切替要求となる。
- 速いが狭すぎる、広いが遅すぎる候補は保持されない。
- no-return後、target不連続、body overlap、置換回数上限では従来どおり切り替えない。

