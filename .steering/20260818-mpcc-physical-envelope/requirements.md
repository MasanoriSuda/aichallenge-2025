# Requirements

## 目的

最新試走 `output/20260818-081035` で確認した、MPCC軌道の事後物理検証失敗と
Recovery保持Missionの再突入を解消し、本来の連続軌道最適化を進められる状態にする。

## 動的根拠

- 18 Mission中12回がPassへ到達したが、`Pass -> Return -> Idle` 完遂は1回。
- Recovery遷移29回中、壁・車体・物理成立性に直接関係するものが26回。
- `optimized horizon failed physical revalidation` が9回。
- Recovery完了後に同じ静的壁不成立を再評価し、
  `Recovery -> FollowPrepare -> Recovery` を反復するepisodeがある。

## 要件

1. Receding-horizon optimizerの各stageへ、実車体と予測path headingを使った
   connected static-wall intervalを与える。
2. optimizerと事後検証、下流MPCが同じ物理wall envelopeを共有する。
3. 事後検証は残し、未知・out-of-map・実接触を緩和しない。
4. Recovery保持後も将来execution horizonがhard infeasibleなら、同じMissionを
   再Recoveryせず終了し、左右候補を現在姿勢から再探索可能にする。
5. 既存のユーザー変更中 `config.yaml` と `aichallenge/result-summary.json` は変更しない。

## Definition of Done

- heading-aware lateral intervalの単体テストが通る。
- Recovery保持Missionのhard horizon不成立が再Recoveryへ入らない経路を持つ。
- package testとAutoware buildが通る。
- 次回走行で次を確認できるログを残す。
  - `optimized horizon failed physical revalidation` の減少
  - `retained pass mission ... infeasible` によるMission終了
  - 同一episodeのRecovery往復回数減少
