# Tasklist

- [x] current-state prefix supervisor条件をpure function化
- [x] active ShiftOut/Pass中のsame-side DP評価を有効化
- [x] target intrusion時もhard constraint付き候補評価を継続
- [x] fresh prefixとlast-feasible tailを結合して全horizon検証
- [x] config/startup/runtimeログを更新
- [x] core unit testを追加・更新
- [x] `make autoware-build`（25 packages成功）
- [x] package test（25/25、1183 tests、0 failure）
- [x] diff確認
- [x] コミット

## Definition of Done

- active execution中にfresh same-side candidateが生成され得る。
- short prefixでもlast-feasible tail込みのcontrol horizonを検証して採用できる。
- 候補不成立で既存active pathが破壊されない。
- hard faultの失効条件が維持される。
- build/testが成功し、`aichallenge/result-summary.json`をコミットしない。
