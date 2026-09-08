# MPCC audit workflow

作業に必要な境界だけ掘り下げる。全見出しのレポート作成を毎回要求しない。
実行権限・方式比較trigger・検証条件は対象packageのAGENTS.mdを正本とする。

## 証拠を固定する

- branch/commitと保存すべき差分
- run ID、Domain、最後の正常と最初の異常のtime/decision ID
- 使用config、build/install、snapshot/replay payloadの対応
- source、unit test、replay、simulation、SIL/HIL、実車の証拠種別

異なるcommit/configのログを一つの時系列にしない。実行できない再現はその理由を残す。

## 失敗までの経路を追う

```text
observation → target/intent → candidate/homotopy → corridor → admission
→ problem → solver → physical certificate → selection → post-processing
→ fallback/recovery → final publish
```

疑わしい境界のowner、context identity、入力時刻、拒否理由、失敗後の行き先を確認する。
特にsemantic stage時刻、観測時刻、control origin、publisher周期を混同しない。
同じtarget IDでもgeneration・course frame・予測tube・terminal意味が一致するとは限らない。

候補仮説ごとに、支持証拠、反証条件、次に必要な観測を記す。
solver失敗より前のproducer欠陥と、後段で問題を隠すretained/hold/retryを分ける。
修正履歴が必要なsymbolだけ`git log -S/-G`やblameで調べ、導入理由・削除条件を確認する。
説明不能なwipはUnknown。正当なEmergencyやretained証明をmaskという名前だけで削除しない。

## 再現と方式比較

同一failure snapshotで元の失敗を再現し、まず疑わしい一つの意味・ownerだけを変えて比較する。
不変のworldを保持し、変更されたproblem/candidate内容には新fingerprintを付ける。
warm-start oracleにはexact QPと実際のwarm startが必要。authority喪失snapshotだけで
それが揃うと仮定しない。snapshot間のsource/壁payloadの同一性を検証する。

package AGENTSのtrigger成立時は、production authorityを固定してA/B/C/D比較を行う。

| 結果 | 調べる境界 |
|---|---|
| A失敗・B成功 | Mission lifecycle |
| A/B失敗・C成功 | candidate生成 |
| A/B/C失敗・D成功 | single-SQP/実時間近似 |
| solverとexact proofが不一致 | model/certificate、時刻・座標・制約の意味 |
| 全失敗、不成立証明なし | Unknown。観測または探索範囲を改善 |

accepted/rejected/inconclusiveとrevisit条件を中央registryへ残す。
局所optimizerが見つけられないことをphysical infeasibilityの証拠にしない。

## 修正と終了

producer修復、consumer検出改善、表現/方式の置換を、削除経路・authority数・
identity/proof・計算時間・移行リスクで比較する。
選んだ修正は失敗するtest/replayと1対1に対応させ、不要となる経路を同時に削除する。
必要なtest/build/replayと動的証拠を取得し、変更・残る問題・rollback commitを報告する。
個別修正の合格と全intent/多車両の統合受入れを区別する。
