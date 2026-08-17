# Design

## 現状

追い越しの1周期は概ね次の順で動く。

1. 離散 Mission / receding-horizon referenceを評価する。
2. 名目 Mission pathに対するruntime wall preplanを評価する。
3. stage corridorをMPC境界へ反映する。
4. progress-contouring MPCCを解く。

このため4で壁境界内の軌道を解けても、その結果は1〜2へ戻らない。次周期も
名目 pathだけでwall escape prefixを要求し、prefixを作れなければ可行なMPCC解を
持ったままMissionを中断し得る。

## 変更方針

`MpcProblem` に、QPへ適用した横境界と追い越しcontextを保持する。solve成功後に
primalのstate stage 1..Nから lateral/progress trajectoryを抽出し、最小境界余裕と
ともに保存する。

次周期は以下をすべて満たす場合だけ解軌道を再利用する。

- ageが短い。
- target ID、Mission generation、side、phaseが一致する。
- 横位置と進捗が有限かつ単調である。
- 現在の進捗へ再sampleできる。
- 現在のstatic-map footprint検証で、hard wall接触を生じず軌道を維持できる。
- target continuity、wall hard guard、emergency guardが正常である。

有効な解軌道は、receding-horizonのwarm-startを実際の制御解へ更新し、名目 path
だけが発生させたsoft runtime wall warningを覆う。hard guardそのものは覆わない。

## 局所リファクタ

- MPCC primalから実行軌道を抽出する純粋helperを `mpcc_progress` に置く。
- controller内では「QP metadata」「保存snapshot」「現在へ整列したsnapshot」を
  分け、Mission FSMへsolver vectorを直接漏らさない。
- telemetryでauthorityの採用・失効理由を確認できるようにする。

## フォールバック

- 抽出失敗、stale、context不一致、再sample失敗: authorityを使わず現行処理。
- 物理再検証失敗: 現行runtime wall preplanを維持。
- solve失敗: snapshotを更新せず、現行solver failure処理。
