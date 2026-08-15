# Design

## 方針

既存FSMはtarget identity、Mission lifecycle、hard fault、Recoveryのsupervisorとして残す。
横参照は次の優先順位にする。

1. 正常なactive execution中に生成したfresh same-side DP prefix
2. 直前のruntime-validated DP path
3. frozen Mission / legacy lateral profile

DP評価は約10 Hz、低レベルのreceding-horizon lateral optimizerとMPCは従来周期で動作する。

## 局所リファクタ

current-state prefixを評価できる条件をpure functionへ分離する。これにより
`FollowPrepare` 後だけでなく、正常な `ShiftOut` / `Pass` も同じhard-gateを通して扱う。

## Partial prefixの扱い

fresh prefixがcontrol horizon末端まで届かない場合、未被覆tailには現在のactive DP参照を
使い、それもなければ現在の実測横位置をholdする。結合horizonが失効した後は既存の
Mission/legacy参照へ戻る。fresh prefixそのものを無限延長はしない。

結合後の全サンプルに対して既存 `evaluate_overtake_line_horizon` を実行し、無修正で
feasibleな場合だけactive pathをatomic更新する。次段のlive optimizerがtarget時系列制約を
毎周期再評価する。

## 影響範囲

- `v2x_overtake_core`: active prefix assessmentの条件判定、atomic promotionの用語整理
- `mpc_controller_cpp`: active executionのshadow評価、tail stitching、ログ
- `config*.yaml`: primary execution有効化、評価周期10 Hz
- `test_v2x_overtake_core.cpp`: supervisor条件とpartial-prefix promotion契約

## 失敗時

新候補が不成立なら現在pathを変更しない。hard fault時は従来どおりauthorityを解除し、
既存のSafety/Recoveryへ委譲する。
