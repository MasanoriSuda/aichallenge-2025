# Requirements

## 目的

壁接触後のStuck Recoveryで車体接触が解消しているにもかかわらず、短い
ForwardCreepの距離が再判定ごとに破棄され、低速動作を反復する事象を解消する。

## 実走根拠

- 対象run: `output/20260817-160322/d1/autoware.log`
- `actual footprint intersects static wall`でRecoveryへ移行した。
- 接触セルが0、`rejoin_safe=1`、`rejoin_path_clear=1`になった後もRecoveryが継続した。
- clear後の各ForwardCreepは0.179～0.195 mで終了し、実質0.20 mの復帰判定へ届かなかった。
- 10 step上限到達後もaggressive retryへ入り、通常走行へ戻れなかった。

## 要件

- 現在footprintがclearな間のForward物理移動を、StopAndReassessを跨いで保持する。
- 接触再発、Reverse開始、Recovery終了、無効なmotion sampleでは累積値を破棄する。
- footprintがclearになる境界を跨いだ1周期分は加算しない。
- 予測停止距離ではなく、妥当性確認済みodometry移動だけを使用する。
- 既存のwall/V2X swept-footprint判定、最大速度、最大試行数は緩和しない。
- ROS 2 interface、評価契約、ユーザーのconfig変更を変更しない。

## Definition of Done

- clear Forward進捗trackerの単体テストが通る。
- 既存package testが通る。
- `make autoware-build`が通る。
- 実走で接触解消後に`LowSpeedRejoin -> Normal`へ戻れることはユーザー試走で確認する。
