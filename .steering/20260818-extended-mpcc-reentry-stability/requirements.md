# Requirements

## 背景

`output/20260818-202140/d1/autoware.log` では、Overtake の ShiftOut / Pass 中に
extended velocity-progress MPCC が OSQP 最大反復へ到達し、failure circuit の cooldown 中は
3-state MPCCへ切り替わっている。少数のsolve failureが多数周期のfallbackと再切替を発生させ、
横経路実行と速度指令の連続性を悪化させている。

## 目的

- extended MPCC失敗後、一度の偶発的な成功だけで制御権を戻さない。
- 複数周期の連続成功をshadow確認してからextended MPCCを再採用する。
- cooldownと再採用判定の責務を分離し、controller本体の分岐を読みやすくする。
- 現行の3-state fail-operational、壁・車両hard guard、Recovery契約を維持する。

## 制約

- ROS 2 topic/service、launch、評価インターフェースは変更しない。
- 追い越しside、Mission、壁余裕、速度上限は変更しない。
- 初回の正常なextended MPCC起動には追加待ちを入れない。
- 再資格確認中も3-state MPCCで走行を継続し、停止fallbackへ落とさない。

## Definition of Done

- failure後は設定された連続成功回数までextended解を制御へ採用しない。
- probe失敗で成功streakとcooldownが正しく再設定される。
- runtime logからrequalification周期と成功streakを確認できる。
- `test_mpcc_progress` と対象packageのbuildが成功する。
