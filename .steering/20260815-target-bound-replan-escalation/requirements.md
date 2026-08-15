# Requirements

## 背景

`output/20260815-154439/d1/autoware.log` では、前方targetを2～6 m残した
早期Returnは解消した。一方、Pass進入13回に対して正常rear-clearは1回で、
次の横経路不成立が支配的だった。

- `optimized horizon escaped target separation bounds`: 5回
- `physical target separation conflicts with wall bounds`: 3回

target-bound execution holdは相対速度を維持したが、代替横経路を即時評価する
契機を持たず、例によってはtarget longitudinalが約0.38 mになるまで接近して
Recoveryへ移行した。

## 必須要件

1. target-bound hold開始時に、次周期の同側・反対側評価を即時要求する。
2. hold中のfresh同側候補は、SafeSeparation中でもatomicに置換可能にする。
3. no-return前のfresh反対側候補は、target-bound failureに限って再選択可能にする。
4. 予測sweepが不成立のまま近距離へ入った場合、target速度未満へ引かず、
   closing speedだけを有限値へ抑える。
5. 壁接触、emergency、target不連続、no-return後の全幅切替は禁止を維持する。
6. 既存のROS topic/service、評価インターフェースは変更しない。

## Definition of Done

- pure policyの単体テストが通る。
- `multi_purpose_mpc_ros`がビルドできる。
- 起動ログで近距離guardの設定値を確認できる。
- 実走で次を確認できるログを出す。
  - target-bound hold開始後の再評価要求
  - fresh同側／反対側置換、または近距離closing guard
  - hard fault時は従来どおりRecovery
