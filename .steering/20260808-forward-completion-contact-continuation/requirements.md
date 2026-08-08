# Requirements

## 背景

`output/20260808-113107/d1/autoware.log`では、P1が`ShiftOut -> Pass`へ13回到達した一方、
`Pass -> Return`は0回だった。Pass中の前方capはほぼ解除されているが、
SafeSeparationが速度参照を前車速度+2.0 m/s付近へ制限し、rear-clear前に
short-horizonまたは局所距離上限で離脱している。

上位走行は横接触後も選択済みの側と前進を維持している。現行は未確定overlapの
短いdebounceしか持たず、確定overlapではcommitted forward completionを失う。

## 必須要件

- 車体非重複かつ確定済みPass corridor上のforward completionでは、前車速度+差分を
  速度上限にせず、通常のコース速度参照を維持する。
- SafeSeparationの局所時間・距離budgetを、固定値だけでなく現在Missionが予測した
  rear-clear必要距離へ合わせる。
- 横接触と判断できる短時間のoverlapでは、target、side、Mission generation、
  forward-completion latchを保持する。
- Contact Continuation中は選択済み側を維持し、前進と相手から離れる小さな横biasを
  同時に与える。
- 正面寄りの衝突、壁接触、target不連続、solver recovery、接触中の進捗停止、
  timeoutは従来どおり中断する。
- `/control/command/control_cmd`などのROS 2契約と評価基盤は変更しない。
- 競技シミュレーション専用の機能として設定で無効化可能にする。

## 非対象

- 接触そのものを目標にする経路生成
- 車両を押す方向への操舵
- 実車向け安全要件の変更
- V2X message、topic、serviceの変更

## Definition of Done

- pure resolverで全速forward escapeとrecoverable side contactを単体確認できる。
- Mission整合budgetが固定12 m未満へ縮まず、絶対Pass上限は越えない。
- Contact Continuationが壁・正面接触・無進捗を抑止しない。
- `multi_purpose_mpc_ros`のbuild/testと`git diff --check`が成功する。

