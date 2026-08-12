# Requirements

## 目的

追い越し中に短い予測区間の再構築が間に合わなくても、現在の経路先端が物理的に安全で前進できている間は、固定の SafeSeparation 距離だけを理由に Pass を破棄しない。

## 対象

- `multi_purpose_mpc_ros` の追い越し SafeSeparation
- 実車体・予測 sweep・壁を用いた安全な trajectory prefix の保持判定
- prefix 保持中の fresh same-side / alternate Mission 再探索
- 速度と横移動を考慮した wall preplan の早期化
- 追い越し episode をログ上で追跡できる識別子

## 制約

- 実接触、壁余裕違反、壁観測不能、EmergencyBrake、solver recovery は従来どおり hard fault とする。
- ContactContinuation の許可条件は緩和しない。
- SafeSeparation の絶対時間・絶対距離上限は延長しない。
- ROS 2 topic / service / message 型と評価成果物契約を変更しない。
- `aichallenge_system` は変更しない。

## Definition of Done

- 安全な prefix と新鮮な前進進捗がある場合だけ、未 latch の Pass が局所上限を再 arm できる。
- prefix が安全でない場合は従来どおり fail closed になる。
- 局所上限の直前に fresh same-side Mission を優先して再利用できる。
- wall preplan が現在位置だけでなく短時間先の車体位置も監視する。
- 主な phase 遷移ログで同一追い越し episode を照合できる。
- core unit test と package build が成功する。
