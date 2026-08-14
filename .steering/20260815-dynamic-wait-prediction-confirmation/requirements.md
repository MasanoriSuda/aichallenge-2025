# Requirements

## 目的

`output/20260815-081746`で確認したDynamicMissionWait中の予測重複チャタリングと、
fresh same-side Pass再開直後のMission期限切れを解消する。

## 対象

- DynamicMissionWaitのforward prefix
- Behavior SafetyBrake仲裁
- fresh same-side Pass continuationのfront-cap handoff
- same-target Mission total budget

## 制約

- 実車向けの速度・加速度上限は変更しない。
- current footprint overlap、壁guard、予測欠損、target jumpは従来どおりfail-closedとする。
- Mission時計をリセットせず、延長は累積上限付きとする。
- `output/`とユーザー変更中の`aichallenge/result-summary.json`は変更・commitしない。

## Definition of Done

- 単発のpredicted overlapでfull closingとSafetyBrakeが周期ごとに反転しない。
- 0.25秒連続したpredicted overlapではfull closing authorityを解除する。
- fresh same-side Pass continuationが確認済みauthorityを引き継ぐ。
- rear-clear予測に必要な場合だけMission期限を延長し、累積上限を超えない。
- `make autoware-build`と対象packageテストが成功する。
