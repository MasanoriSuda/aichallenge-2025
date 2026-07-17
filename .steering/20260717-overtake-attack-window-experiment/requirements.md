# Requirements: Overtake Attack Window Experiment

## Goal

6周・着順勝負のSIMレースで、短い直線における追い越し開始機会と完遂率を増やす。
既存のSafetyBrake、車体非重複幅、内側ヘアピン禁止、hard waypoint禁止は維持する。

## Scope

- 追い越し速度参照をShiftOutとPassで分離する。
- 新規追い越し時に、次のhard curveまでに前車を抜ける距離があるか確認する。
- V2X障害物の予測時刻をwaypoint indexではなく経路距離と参照速度から求める。
- 前方2台でも外側corridorを評価する。2台間のgapは引き続き禁止する。
- soft curveでは既に開始した外側追い越しだけ継続可能にする。
- 追い越し横目標を平滑化し、solver abortまでの連続失敗猶予を増やす。
- 判断根拠を既存V2X debug logへ追加する。

## Out of Scope

- 共通センターラインへの全車両s/d射影と匿名track ID生成。
- 動く2台の間を通過するvehicle-vehicle gap。
- SafetyBrake、車体半幅、ROS topic/service、評価JSONの変更。
- 同一制御周期内のOSQP soft retry。`init_problem()`がFSM状態を更新するため、再入可能化を
  別作業で行ってから実装する。
- 壁marginの全面的な縮小。今回の実験では物理安全余裕を維持する。

## Acceptance Criteria

1. ShiftOut中は前車に対するclosing-speed capを維持し、Pass中はそのcapを外して元の軌道速度参照を使う。
2. 新規追い越しはhard curveまでの利用可能距離が推定必要距離以上の場合だけ開始する。
3. V2X予測時刻は各path segmentの`distance / predicted speed`の累積値になり、設定上限で飽和する。
4. multi-front有効時も`v2x_vehicle_vehicle_gap_enabled: false`を維持する。
5. hard WP、内側追い越し、EmergencyBrakeは従来どおり拒否される。
6. pure helperの境界条件を単体テストで確認し、`make autoware-build`が成功する。
7. `make dev3`では壁・車両接触、Fatal、長時間停止を増やさず、追い越し開始・継続理由をログで確認できる。

