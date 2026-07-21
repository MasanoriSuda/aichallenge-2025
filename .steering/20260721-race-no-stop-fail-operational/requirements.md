# Requirements

## Goal

`make dev3` のレース中に、MPC solver failure や接触後の復旧候補不成立を理由として車両が恒久停止する経路を減らす。

## Scope

- 変更対象は `multi_purpose_mpc_ros` とその設定・単体テストに限定する。
- fail-operational 動作は simulation time 使用時だけ有効にする。
- 前方車を検出している場合や SafetyBrake 中は solver-failure crawl を許可しない。
- 障害物停止時は Reverse-first を維持するが、Reverse 不成立後の Forward 探索を禁止し続けない。
- 接触中の緩和候補は、接触セル数を増加させない bounded step に限定する。
- ROS topic、message、service、Domain 契約を変更しない。

## Definition of Done

- 前方車なしの MPC solver failure に対して、ゼロ速度まで減速する代わりに設定可能な低速参照線追従を出力できる。
- 停止中の solver failure では、AWSIM の小さな pose/progress 補正で復旧確認時間がリセットされない。
- coordinated stop の初回 Reverse が不成立なら、次の bounded retry で Forward 候補を探索できる。
- 接触改善候補がない場合、SIM aggressive mode では非悪化候補を探索できる。
- 単体テストと `make autoware-build` が成功する。
