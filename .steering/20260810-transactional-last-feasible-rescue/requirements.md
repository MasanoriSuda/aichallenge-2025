# Requirements

## 目的

追い越し中の一時的な予測・候補欠落で、直前まで成立していた Mission を安全に再利用できるようにする。

## 必須要件

- 単発の soft miss では last-feasible cache を消去せず、TTL 内は保持する。
- target/generation 不一致、target discontinuity、非回復接触、壁・solver の hard fault では再利用しない。
- cache の freshness は時刻だけでなく、ego/target の移動量でも判定する。
- Mission 置換は prepare -> validate -> commit の順で行い、失敗時に旧 Mission を変更しない。
- 回復可能な横接触は、単独では hard fault として扱わない。
- ROS topic/service、提出物、評価基盤の契約は変更しない。

## 対象外

- side replacement 回数上限の変更
- 追い越し速度・壁余裕の攻撃化
- フル MPCC/軌道最適化への置換

