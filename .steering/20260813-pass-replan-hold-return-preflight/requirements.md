# Requirements

## 背景

`output/20260813-224307` では、16回の ShiftOut のうち15回が Pass へ到達した一方、
正常な `Return -> Idle` は2回だけだった。主な失敗経路は次の二つ。

- MPCC-lite shadow は current-side hold を feasible と評価しているのに、実行層の
  target separation bound 不成立で Pass を破棄して DynamicMissionWait へ移る。
- rear-clear 後に Return を開始してから、static wall clamp と横加速度制約の不整合で
  Recovery へ落ちる。

## 要求

1. Pass中のtarget-bound不成立が物理接触・壁異常・EmergencyBrakeを伴わない場合、
   Missionとsideを保持したまま短時間だけ物理的に成立する同側軌道を実行する。
2. 保持中も毎周期MPCC-liteを再評価し、新しい解が成立したら即座に通常実行へ戻す。
3. 保持中はFollow速度へ落とさず、進入時の実速度を下限として前進を維持する。
4. 保持は既存のcontinuity leaseとPass hold距離で有界化し、hard faultを迂回しない。
5. Returnは現在速度・横位置・壁形状・横加速度を使って事前検証し、必要ならReturn距離を
   既存上限内で延長する。成立しないReturnはPassから開始しない。
6. SafeSeparationの局所的・短時間な不成立はMissionを破棄せず、Pass内再計画へ戻す。

## 制約

- V2X、control command、launch、評価インターフェースを変更しない。
- 壁接触、壁計測不明、EmergencyBrake、target discontinuityは従来どおりhard fault。
- Recovery、Reverseの優先順位は変更しない。
