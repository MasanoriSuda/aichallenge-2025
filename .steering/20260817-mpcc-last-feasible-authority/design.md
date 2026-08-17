# Design

## 観測した問題

`20260817-191414` では、MPCC/DPの実行authorityが有効になっても約0.3秒で解除され、
再び有効になる遷移を繰り返している。また、MPCC solve成功後にも
`rejected during extraction` が断続的に発生する。

現行実装には次の不整合がある。

1. 実行軌道抽出の許容差が固定 `1e-5` で、OSQPが受理したconstraint residualより
   厳しい場合がある。
2. 保存進捗がphase単位なので、ShiftOutからPassで進捗が0へ戻り、同一Missionの解を
   context mismatchとして捨てる。
3. 保存解の最大ageが0.15秒で、一時的な抽出失敗を十分に橋渡しできない。
4. QPへ適用済みのstage wall boundを物理再検証時にもう一度wall clearance分だけ
   縮めている。
5. 保存解はwarm-startとsoft wall warning抑制に使われるが、DP authority lease欠落時の
   実行継続には使われない。

## 変更方針

- 抽出helperへ失敗理由とstageを返す診断値を追加する。
- solve結果の最大constraint violationを抽出許容差へ反映する。
- 保存基準をphase進捗からMission累積進捗へ変更する。
- 許可するphase handoffは `ShiftOut -> Pass` と同一phaseだけに限定する。
- last-feasible解は0.35秒まで保持する。ただし利用時は毎回、現在のstage boundと
  static-map footprintで再検証する。
- QPへ既に適用したstage boundは追加で縮めず、static-map footprint側だけでhard
  clearanceを再確認する。
- terminal stageのheadingは直前segmentから推定し、ゼロ固定による偽の壁接触を防ぐ。
- DP authorityが欠けた周期だけ、再検証済みlast-feasible軌道を実行authorityとして
  使用する。

## フォールバック

保存解がstale、別context、進捗退行、再sample不能、現在の壁接触、target hard guardの
いずれかなら橋渡しを行わず、既存のMission/Recovery処理へ戻す。
