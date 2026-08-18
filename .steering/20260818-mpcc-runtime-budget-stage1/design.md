# Design

## 1. Bounded horizon refresh

`control_rate=40 Hz`は維持する。完全なreceding-horizon solveは最大0.10秒だけ間引き、間の周期は直近のhard-validated evaluationをそのまま使用する。WPや計画contextが変化する区間では40 Hzでもfresh solveを許す。

再利用は次の全条件を満たす場合だけ許可する。

- 同じreference waypoint
- 同じtarget / Mission generation / phase / side
- 最終fresh solveから0.10秒以内
- 現行continuity leaseの全hard guardを通過
- 新しいpredicted-overlap replan要求がない

同一WPに限定することでstage indexとstatic-map検証位置のずれを作らない。WP更新、phase遷移、Mission置換では40 Hz周期を待たずfresh solveへ戻る。

## 2. Conditional RTI-SQP

first QP solutionは常に採用候補として保持する。2回目は、以下のいずれかが成立し、かつfirst solve終了時点で計算予算内の場合だけ実行する。

- lateral bound reserveが小さい
- first solutionとlinearization pointの横位置・姿勢差が大きい
- horizon内に大曲率がある

2回目が失敗した場合は従来どおりfirst feasible solutionを使用する。

## 3. Static wall cache

- heading bucketを0.01 radから0.025 radへ統合する。
- bucket内最大heading差をfootprint marginへ加える保守性は維持する。
- clearanceを1 cm単位で上向き量子化する。
- capacityを16384件へ増やす。
- `unordered_map.begin()`の任意削除を廃止し、deterministic LRUで再利用中entryを保持する。

## 4. Telemetry

既存のcallback、OSQP、wall cacheログへ加え、horizon scheduleのfresh/reuse件数とRTI skip理由を1秒窓で出す。

## 今回行わないこと

- core MPCC solverの別thread/process化
- command 40 Hz / core solve 20 Hzの完全分離
- 追い越しマージンや攻撃性の変更
- hard wall / current footprint制約の緩和

完全分離は本Stage 1の実走結果を確認した後のStage 2とする。
