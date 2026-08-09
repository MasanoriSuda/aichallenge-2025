# Requirements

## Purpose

最新走行で確認された、実行不能になった同一 Overtake Mission を
`Pass -> FollowPrepare -> Pass` で繰り返し再開するループを止める。

## Scope

- 実行中に不成立となった Mission generation を失効させる。
- 失効 generation は同じ固定経路のまま再開しない。
- 完全成立した反対側 Mission があれば generation を更新して置換する。
- 反対側も成立しなければ Recovery で基準線へ戻り、Mission を終了する。
- rear-clear までに全幅切替が必要なのに、その切替経路を事前検証できていない
  初期候補を採用しない。

## Constraints

- 速度・加速度・壁クリアランスの設定値は変更しない。
- ROS 2 topic / service / message 契約を変更しない。
- 接触継続や SafetyBrake の hard fault 条件を緩和しない。
- 既存のユーザー変更を巻き戻さない。

## Definition of Done

- 失効 generation が `ResumeCurrent` されない単体テストが通る。
- 未検証の全幅切替を必要とする候補が拒否される単体テストが通る。
- `multi_purpose_mpc_ros` がビルドできる。
- 対象 package のテストが通る。

