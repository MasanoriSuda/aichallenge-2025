# 設計

## 現象

現行`PassOuterHorizon`は最初の有意曲率からouter strategyを分類し、固定した
`pass_side_sign`が後続カーブのinner sideと一致すると
`outer pass becomes inside before rear-clear`で不成立にする。

これは問題の検出にはなるが、外側経路を再生成しない。そのため候補棄却、
SafeSeparation、Recoveryが増え、通過可能な次の外側へ移れない。

## 1. rolling outer request

Pass中に現在位置から設定距離先までの基準曲率を調べる。有意曲率
`|kappa| > threshold`に対する外側は次とする。

```text
inner_side = sign(kappa)
outer_side = -inner_side
```

現在sideと異なるouter sideが一定走行距離以上連続するときだけreplanを要求する。
直線と短い曲率ノイズは前の判定を変えない。

## 2. side transition gate

反対側への横断は次をすべて満たす場合だけ行う。

- Pass active、outer strategy committed
- target observation continuous、position jumpなし
- current body footprint非重複
- short horizon hard-safe、fresh predictionあり
- target centerが設定距離以上前方
- 予測上、横移動完了時にもtargetが侵入guard距離より前方
- 前回切替からcooldown経過
- 最大切替回数未満

target前方距離を要求するのは、side-by-sideで相手の軌跡を横断しないためである。

## 3. validated lateral transition

既存のPass extension plannerを拡張し、replacement sideを指定可能にする。

1. 反対側でtarget center separationを満たすgoalをwall interval内に生成
2. 横加速度から必要shift distanceを計算
3. 次の反転カーブへ入る前にshiftを完了できることを確認
4. target速度、closing speed、curve speed capを使いrear-clearまでrollout
5. rollout上もshift完了までtargetが前方に残ることを確認
6. static wallと実行horizonをpreflight
7. target/generation/prediction ageをatomic commit直前に再確認

commit後は`pass_side_sign`、fixed goal、frozen PassPlanを同時更新する。横移動中は
front-cap latchを解除し、物理横離隔が再成立するまでunlatched Pass速度を使う。
横移動は検証済みdistance-domain rampが所有し、通常のcycle単位goal slewを二重適用しない。

## 4. initial admission

rolling replanが有効な場合、将来の曲率符号反転だけでは初回missionを棄却しない。
wall、target separation、body deadline、rear-clear、横加速度のhard guardは維持する。
実行中に反転区間がlookaheadへ入った時点で上記side transitionを再検証する。

## 5. Non-goals

- MPPIへの全面置換
- 横並び中の強制side crossing
- wall/contact/Emergency guardの緩和
- `a_max`、車体寸法、自己位置推定の変更
