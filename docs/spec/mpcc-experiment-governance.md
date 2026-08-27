# MPCC architecture experiments

> 現行設計の正。2026-08-28時点。2025 AWSIM由来の競技制御を2026向けに整理するための開発手順であり、2026公式制御方式ではない。

## 目的

単一normal authorityを維持しながら、persistent Mission、候補生成、MPCC定式化、逐次凸化、solverを交換可能な実装仮説として比較する。症状ごとの例外処理を増やす前に、同一failure snapshotで方式の限界を判定する。

## Algorithm Pivot Gate

対象packageの`AGENTS.md`に定義したtriggerが成立したら、production変更とパラメータ調整を停止する。比較中はauthority、clearance、weight、solver tolerance、timeout、lease、grace、fallbackを変更しない。

比較順序は次のとおり。

1. AとBでpersistent Mission lifecycleを分離する。
2. BとCで候補生成を分離する。
3. CとDでlive single-SQP／実時間近似を分離する。
4. numerical solveとexact proofが異なる場合はmodel/certificate mismatchとして別に扱う。

## Failure snapshot

snapshotは次をimmutable payloadとしてSHA-256でsealする。

- seven-state、control origin、直前publish command
- reference path／stage geometry
- wall grid／footprint／hard clearance
- active targetと全relevant peerの観測・予測tube
- encounter、target、homotopy、commit/no-return
- exact QP／SQP problemとsolver schema
- warm startとasync provenance
- exact physical trajectory／certificate入力

ログだけでこれらを再構成できない場合は`replay_ready=false`として記録し、A--D証拠へ使わない。

## 判定

| 結果 | 分類 |
|---|---|
| A失敗、B成功 | Mission lifecycle defect |
| A/B失敗、C成功 | candidate generation defect |
| A/B/C失敗、D成功 | single-SQPまたは実時間近似の限界 |
| solve成功、proof失敗 | model/certificate mismatch |
| 全失敗＋不成立証明 | physical infeasibility |
| 全失敗、証明なし | Unknown |

## Experiment registry

正本は`docs/spec/mpcc-experiment-registry.json`。採用だけでなく棄却と保留も保存し、同じ試験を再提案できる条件を`revisit_condition`へ明記する。

## Slice 7 Gate

Slice 7のparameter tuningは、少なくとも次を満たした後にだけ開始する。

- normal publisher authorityがcanonical seven-stateだけである。
- Track/Cruise、Follow、ShiftOut、Pass、Returnの動的受入れがある。
- stale/wrong-generation/unproved artifact publicationが0件である。
- failure snapshotが再生可能で、残る異常が構造欠陥ではなく性能差として分類される。

## Slice 7 bounded tuning outcome（2026-08-28、2025由来の暫定）

canonical seven-state normal authorityの動的基準はcommit `b273d56d`、
`output/20260828-044759`、`N=20`、40 Hz production solve submissionとする。
同runでは`Idle -> ShiftOut -> Pass -> Return -> Idle`が完遂し、Overtake
Recoveryおよびactual-footprint wall-margin violationは0件だった。

この基準に対し、`N=16`、`N=18`、`N=20`のままproduction solveを20 Hzへ
間引く三候補を一変数ずつ評価した。いずれも計算量の一部は改善したが、
独立runでterminal successor、wall feasibilityまたはRejoin authority continuityを
失ったため棄却し、source/config/testを基準状態へ戻した。

したがってbounded Slice 7 tuning campaignは「採用parameter変更なし」で完了する。
これはrace-production品質の完了を意味しない。callback tail、maximum iteration、
canonical Emergencyはarchitecture/integration backlogとして扱い、proof horizon短縮、
solve間引き、clearance、tolerance、leaseで隠さない。再試験条件は
`mpcc-experiment-registry.json`を正本とする。
