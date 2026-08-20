# Requirements

## 目的

`20260820-221931` の試走で残った、物理壁判定により棄却した追い越し側を直後に再選択し得る欠陥と、solver復帰後のwall handoffが長時間減速を継続する事象を修正する。

同時に、計画時のFrenet回廊余裕と実行時の車体footprint壁距離を同一指標のように表示していたログを改め、経路採用後のどこで壁契約が破れたか追跡できるようにする。

## 対象

- ShiftOut / Pass中に最終予測経路が物理壁契約を満たさなかった場合の同一target・同一side再試行
- bounded solver continuationから通常制御へ戻るwall handoff
- wall admission決定ログの計画値・実行値の意味

## 制約

- ROS 2 topic、message、launch、提出インターフェースを変更しない
- 現在footprintまたは予測経路が危険な場合に、安全確認なしで制御を解放しない
- 追い越しクリアランス等のconfig値は変更しない
- `output/`、result JSON、crash artifactは変更・コミットしない

## Definition of Done

- 物理壁判定で失敗したtarget/sideへ既存のretry cooldownを適用する
- no-return前なら反対側の探索を妨げず、失敗側の即時再採用を禁止する
- solver復帰時は、その周期の新しい物理予測が安全なら1回の確認でhandoffする
- 危険または取得不能な予測は従来どおりholdし、unsafe timeout releaseを作らない
- ログで `planner_metric=frenet-corridor-reserve` と `execution_metric=physical-footprint-distance` を区別する
- unit test、package build/testが成功する
