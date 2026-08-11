# Requirements

## 目的

追い越し性能を変更せず、次の性能修正を小さなpure policy変更として実施できるよう、
開始段階とPass終盤の判定責務を局所的に整理する。

- 微小横移動MissionをDirectPassとして扱う修正
- rearward contact / SafeSeparationからrear-clearまで完遂する修正
- entry pre-armのauthority追加

## 観測した構造上の問題

- Idle / FollowPrepareからの開始phase決定が、controller内の複数boolと優先順位付き
  三項演算子へ分散している。
- targetが後方へ移った後の完遂資格が、ContactContinuationとSafeSeparationで
  個別に組み立てられている。
- 現状の挙動を変える前に、分類結果と共通資格を単体テストで固定する必要がある。

## 必須要件

1. 新規Mission、base-line DirectPass、pause resumeの開始phaseとreasonをpure policyへ分離する。
2. target rearward、SideBySide commit、target continuity、forward latch、body separation、
   predictionの共通文脈をpure policyへ分離する。
3. controllerはpure policyの結果を使用し、現行のphase、reason、安全条件を維持する。
4. config値、ROS 2 interface、launch、topic、message型を変更しない。
5. `aichallenge/result-summary.json`を変更しない。

## 完了条件

- entry分類の全優先順位をpure core testで固定する。
- rearward completionのcontact/separated/physical/progress境界をpure core testで固定する。
- 既存のcore testと対象package buildが成功する。
