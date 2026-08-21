# Requirements

## 背景

直近試走では、DynamicEscapeの主候補について将来の壁実行preflightが不成立に
なることは検出できていた。一方、反対側候補の評価は主候補が完全に不成立となった
後にしか開始されず、横へ逃がせる時間を失ってからFollow/停止へ移る経路が残っている。

また、反対側候補の試算がlive GapPlannerの継続状態を更新するため、反対側を不採用に
しても次周期の主候補へ影響し得る。候補生成、比較、採用のライフサイクルが分離されて
いないことが、左右選択の再現性と原因解析を悪化させている。

## 目的

1. 主候補がhard failureになる前に、将来の狭窄・壁margin継承を検知する。
2. 将来リスクがある場合だけ反対側候補を予防評価する。
3. 反対側が明確に低リスクなら、停止前にそちらを採用する。
4. speculativeな反対側評価とlive GapPlannerの継続状態を分離する。
5. 同一attempt IDで、予測、反対側評価、棄却、採用、authority移譲を追跡可能にする。

## 制約

- ROS topic、message、launch、提出インターフェースを変更しない。
- 壁余裕、速度、Recoveryの設定値を変更しない。
- 既存のhard wall/vehicle guardを緩和しない。
- `aichallenge/result-summary.json`の既存変更を触らない。
- active Passのno-return後に全幅切替を追加しない。本変更はFollow中のDynamicEscape入口に限定する。

## Definition of Done

- 主候補の将来reserve不足、margin-escape、tracking wall contractを分類できる。
- hard failure前でも必要時だけ反対側を評価する。
- primary/alternateのrisk tierを比較し、alternateが明確に良い場合だけ採用する。
- 不採用のspeculative候補がlive GapPlannerのcontinuity stateを汚染しない。
- 決定ログにthreat reason/distance/reserve、評価理由、選択理由が一行で出る。
- 対象packageのbuild/testが成功する。
