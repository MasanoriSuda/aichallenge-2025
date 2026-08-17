# Requirements

## Goal

事故等で停止または低速化した前方車に対し、通常 Overtake planner が完全な
ShiftOut / Pass / Return Mission を生成できた場合、その経路を停止車専用 planner
や Stuck Recovery に捨てさせず、直ちに追い越しへ引き継ぐ。

## Scope

- 停止車専用 planner と通常 OvertakeLine の制御権仲裁
- Stuck Recovery から検証済み即時 Overtake Mission への引き継ぎ
- 純粋関数の単体テスト

## Constraints

- 壁接触、solver failure、V2X 不完全、衝突悪化、Reverse-only Recovery は従来どおり Recovery が所有する。
- 車速閾値や壁余裕を一括で攻撃化しない。
- ROS 2 topic / service / message 契約を変更しない。
