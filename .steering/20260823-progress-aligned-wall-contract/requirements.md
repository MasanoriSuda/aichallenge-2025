# Requirements

## Objective

Overtake canonical MPCCの壁制約を、固定stage indexではなくMPCCが解いたcourse progressと同じ物理位置へ対応させる。QPで許可した解が、同じcycleの実車体証明で壁接触として棄却される構造的不整合を解消する。

## Constraints

- wall margin、solver tolerance、速度、追い越し開始条件は調整しない。
- 物理証明を緩めない。
- retained解、fallback、timeout、lease、feature flagを追加しない。
- Track/Cruise・Followのproduction authorityを退行させない。
- Overtakeのtarget corridorとwall-only corridorの責務を混同しない。
- ユーザー変更 `aichallenge/result-summary.json` は編集・stage・revertしない。

## Acceptance

- stage固定壁境界とsolved progressの対応関係が型または明示的契約で表現される。
- Overtake extended QPの壁制約がsolved progressへ再線形化される。
- 最終解は従来どおり実車体・実地図で証明される。
- 同一bag replayで、原実装の `HardWallContact` 5件および一回box差し替え試作の
  7件が減少し、Track/Cruise solve failureを増やさない。
- build/testが通る。
