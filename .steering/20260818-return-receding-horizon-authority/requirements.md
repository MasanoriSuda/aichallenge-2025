# Requirements

## 目的

Pass で rear-clear を獲得した後、Return が固定 6 m の事前計算参照へ急に切り替わり、
大きな heading error と壁 Recovery を起こす現象を抑える。

## 実走根拠

- `output/20260818-074358` では `Pass -> Return` が 2 回発生した。
- 2 回とも `Return -> Idle` へ完了せず `Return -> Recovery` となった。
- Return 開始時に 6 m の preflight reference が固定され、その直後に
  `MPCC solved trajectory released execution authority: reason=warning inactive`
  が記録された。
- 失敗時は `epsi=-0.559 rad` または static wall footprint/margin infeasible が記録された。

## 必須要件

1. Return を横軌道 receding-horizon 最適化の実行phaseに含める。
2. progress-contouring QPから抽出した実行軌道を Return 中も記録・再利用する。
3. `Pass -> Return` の短いphase境界で、直前のfeasible解をwarm startとして引き継ぐ。
4. rear-clear確定済みのlocked targetはReturn横制約から外す。
5. 壁footprint、Return corridor blocker、EmergencyBrake、solver hard faultは緩和しない。
6. ROS 2 topic/service、launch、評価schemaを変更しない。
7. ユーザー変更中の `config/config.yaml` と `aichallenge/result-summary.json` は変更しない。

## 非目標

- Return距離や壁余裕パラメータの再調整
- Recovery/Reverseロジックの変更
- 全追い越しFSMの全面MPCC置換
