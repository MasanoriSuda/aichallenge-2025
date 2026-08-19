# Requirements

## 対象

`output/20260820-081154/d1/autoware.log` で再現した、追い越し開始後の
Mission 失効と失速を対象とする。

## 観測した欠陥

1. Pass 中に pre-contact squeeze が `bias=0.15 m`、`wall_limited=0` で
   横逃げを開始しても、同じ周期の front-cap 再適用で Pass の幾何所有権を
   失い、次周期に Mission が破棄される。
2. ShiftOut 中に failover を開始した時点では反対側切替が可能でも、非同期の
   左右評価を待つ間に no-return が latch され、結果受信前に切替権限を失う。
3. runtime-failover ログは候補の有無しか示さず、候補が不成立になった理由と
   一時的な切替 lease の有無を判別できない。

## 制約

- 新規追い越し入口の completion-proof gate は緩和しない。
- 壁接触、EmergencyBrake、target discontinuity、solver recovery は従来どおり
  hard fault とする。
- no-return 後の無条件な反対側横断は許可しない。failover 開始前に許可されて
  いた場合だけ、短い時間・距離 lease 内で一度評価できるようにする。
- ROS 2 topic、message、launch、評価インターフェースは変更しない。

## Definition of Done

- wall-bounded pre-contact escape が有効な間は、front-cap の latch 解除だけで
  Pass 所有権を失わない。
- failover 開始時に成立していた cross-side 権限を、既存 wait budget 内で
  非同期評価へ引き渡せる。
- runtime-failover ログから trigger、current/alternate の理由、lease、横逃げ
  所有権を一行で照合できる。
- package build と既存・追加 unit test が成功する。
