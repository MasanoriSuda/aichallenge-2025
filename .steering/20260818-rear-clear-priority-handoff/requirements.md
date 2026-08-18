# Requirements

## 目的

Pass中に相手がすでに後方へ移り、rear-clear成立が目前であるにもかかわらず、
将来のMPCC軌道再検証失敗でFollowPrepareへ戻って実行権限とwarm startを失う挙動を止める。

## 対象

- Pass中のtarget/physical receding-horizon failure
- rear-clear確認直前の短いcurrent-side hold
- rear-clear確認後の既存Return handoff

## 制約

- SideBySideCommittedかつforward-completion済みのMissionだけを保持する。
- target continuity、現在車体分離、予測または新鮮な前進実績を必須とする。
- 現在横位置を維持する短期経路をwall/footprint/lateral-accelerationで再検証する。
- wall contact/margin/sample異常、EmergencyBrake、solver Recovery、forbidden waypointは上書きしない。
- rear-clear距離と確認時間は既存設定を変更しない。

## Definition of Done

- rear-clear直前の将来軌道失敗でPass authorityを維持できる。
- rear-clear確認後は既存のvalidated Returnへ移行する。
- hard faultまたはcurrent-side経路不成立では従来のreplan/Recoveryを維持する。
- 関連unit testと`make autoware-build`が成功する。
