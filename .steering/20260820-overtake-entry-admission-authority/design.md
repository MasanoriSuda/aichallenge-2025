# Design

## 方針

ファイル分割を目的にせず、progressive prefixを実行可能にする判定を
`resolve_mpcc_lite_prefix_execution()`へ集約する。この純粋関数は従来の
body-clear、target、wall、速度、時間・距離budgetに加え、新規entryの場合だけ
`resolve_progressive_entry_completion_gate()`を適用する。

## 実行契約

### 新規progressive entry

- `new_entry_context=true`
- `candidate_progressive=true`
- completion-proof入力が妥当
- front-distance reserveを満たす
- no-returnまでの時間reserveを満たす
- 既存のprefix physical admissionを満たす

以上をすべて満たす場合だけ採用する。

### complete Mission

complete Missionはrear-clearまで予測済みなので、progressive completion-proofの
対象外とする。既存の完全Mission admissionを維持する。

### 実行中Missionのprefix更新

`new_entry_context=false`の同側rolling replanは、現在のMissionを継続する処理で
あり、新規entry gateは適用しない。既存のno-return、SafeSeparation、physical
admissionを維持する。

## ログ

MPCC entry authorityの最終結果に次を出す。

- `source=dual-current|cached-fresh`
- `candidate=progressive|complete`
- `admitted=0|1`
- `prefix_reason`
- `completion_checked`
- `completion_reason`
- `front/current-required`
- `closing`
- `time_to_no_return/current-required`

拒否ログは状態変化またはthrottleで抑制し、周期ごとの大量出力を避ける。

## 互換性

変更は参加者パッケージ内部のC++型、判定、ログ、テストに限定する。
ROSインターフェースと評価成果物schemaへの影響はない。
