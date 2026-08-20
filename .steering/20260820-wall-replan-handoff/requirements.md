# Requirements

## 目的

直近の本試走で確認した、追い越し経路の物理壁余裕不足を検出した後に、同じ経路と直前操舵を保持したまま減速し、停止へ至る連鎖を解消する。

## 対象

- ShiftOut / Pass の実行中に、最終出力側の実車 footprint scan が経路の壁契約違反を検出した場合
- planner が報告した壁余裕と最終物理 scan の壁余裕が一致しない場合の診断
- 壁NG後の Mission generation、再計画要求、最終制御 authority の追跡

## 制約

- ROS 2 topic、message、launch、提出インターフェースを変更しない
- 壁接触中または現在 footprint が不正な場合は従来どおり停止側に倒す
- 設定値の攻撃化は行わない
- `output/`、result JSON、crash artifact は変更・コミットしない

## Definition of Done

- 未来経路だけが壁NGで現在 footprint がclearなら、現在Mission generationを失効させてDynamic Mission Waitの再計画へ移る
- 同じ危険な frozen Mission / physically validated trajectoryを再利用しない
- 再計画中のwall admissionは安全な新経路が連続確認されるまで保持する
- ログ1行から planner/physical wall余裕、差分、Mission generation、replan要求を追跡できる
- orchestrator unit testとpackage buildが成功する
