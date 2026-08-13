# Design

## 方針

直前周期に生成・物理再検証された receding-horizon を、同一 Mission generation と pass side に限って `execution lease` として扱う。lease は既定0.30秒で、更新できなければ自動失効する。

## 1. Execution lease

lease 成立条件は次のANDとする。

- receding-horizon と continuity が有効
- ShiftOut または Pass
- frozen Mission と有効な side がある
- 最後の可解解と現在の generation/side が一致
- 最終可解時刻から設定時間以内
- position jump、course progress reject、corridor block、EmergencyBrake、solver recovery、明示禁止 waypoint がない

Behavior では、この lease を既存の body-clear handoff / Pass latch に追加する。line FSM ではさらに、actual wall hard fault と直近の robust body/prediction clear を要求する。

## 2. SafeSeparation integration

SafeSeparation が始まった後でも、直近の可解経路と安全な予測が lease 内なら `safe trajectory prefix` と同等の前進権限を与える。既存の `full_speed_forward_escape_enabled`、絶対Pass時間・距離上限、rear-clear判定は維持する。

短いV2X欠落では line state に保持済みの longitudinal/speed と直近clear時刻を使う。leaseを超えた欠落は従来どおり失敗とする。

## 3. Last feasible horizon hold

optimizer 本体だけが一時的に失敗した場合、warm start として保存された直前解を現在のsample boundsへ射影し、wall/lateral-accelerationを再検証する。再検証を通った場合だけ出力する。

以下は保持しない。

- sample input/bounds不正
- target-side hard bound不成立
- 再検証後のwall/lateral execution infeasible
- lease期限切れ

## 4. Return ownership

runtime wall warning による speed-preserving Return は `FinishReturn` を明示する。Return途中の同一target再捕捉は、通常の戦術的Returnにだけ残す。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: lease判定の純粋関数
- `mpc_controller_cpp.cpp`: 状態、Behavior所有権、SafeSeparation連携、fallback保持、ログ
- `config.yaml`, `config_for_cloud.yaml`: continuity enable/lease
- `test_v2x_overtake_core.cpp`: lease hard guard/expiryテスト

