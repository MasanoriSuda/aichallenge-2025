# Requirements

## 目的

壁接触中にForward escapeが繰り返し失敗しても同じForward候補を再採用し続け、
Reverseへ切り替わらないRecovery固着を解消する。

## 実走根拠

- 対象run: `output/20260817-164553/d1/autoware.log`
- Overtake Pass後に`actual footprint intersects static wall`でRecoveryへ移行した。
- 壁接触セルが24から47まで増え、`clear_forward=0`のままForwardを反復した。
- `aggressive_forward_retry_limit_before_reverse=2`にもかかわらず、Reverse切替ログがなかった。
- Forward失敗理由は`collision_worsening`または`forward_duration_limit`だったが、
  aggressive retry判定時には`escape_step_limit_reached`へ上書きされていた。

## 要件

- Forward maneuverの失敗を発生時点で記録し、SafeStopとstep再判定を跨いで保持する。
- `collision_worsening`と`forward_duration_limit`をForward失敗として扱う。
- 1 aggressive retry cycle内の複数step失敗は1回として数える。
- 設定回数へ達した次cycleはReverse候補だけを許可する。
- Reverse実行、Forward escape成功、Recovery終了、新episode開始で失敗履歴をresetする。
- wall/V2X rollout、gear確認、速度・距離上限は緩和しない。
- ユーザーのconfig変更とROS 2 interfaceを変更しない。

## Definition of Done

- pure C++ trackerの単体テストが成功する。
- `make autoware-build`が成功する。
- `multi_purpose_mpc_ros`の全testが成功する。
- 実走で2 cycleのForward失敗後に`next_direction=Reverse`が出ることはユーザーが確認する。
