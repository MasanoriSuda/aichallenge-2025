# Tasklist: OSQP row-contract root-cause audit

- [x] 既存変更と前Sliceの棄却結果を確認する
- [x] OSQP wrapperの終了判定とrow residual計算を読む
- [x] 5状態QPのconstraint layoutと後段normalizerを追跡する
- [x] failure-first testでglobal tolerance leakageを再現する
- [x] row indexをcategory/stageへ割り当てる
- [x] 相対許容誤差無効化、row normalization、dual provenanceを比較する
- [x] `make autoware-build`（25 packages）
- [x] package test全件（1635 tests、0 failure）
- [x] 1周超の無人動的確認
- [x] audit結果と不採用理由を更新する
- [x] 不採用実装を削除する
- [x] audit documentだけをcommitする

## Definition of Done

- 原因と症状の因果をコード・test・run logで説明できる。
- `execution-primal-reject` を別名のfallback/rejectへ移しただけではない。
- row契約違反の解をproductionへ採用しない。
- solve failure、overrun、wall/contact proof rejectを悪化させない。
- 不要になったglobal-only acceptanceまたは重複検査を整理できる。

このDoDは候補B/Cでは成立しなかった。特にwall/contact proof rejectと
maximum iterationが悪化したため、production codeへは残さない。
