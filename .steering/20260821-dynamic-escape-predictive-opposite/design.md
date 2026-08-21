# Design

## 1. Candidate lifecycleの分離

DynamicEscapeのcandidate処理を次の4段階として扱う。

```text
generate
  -> forecast
  -> compare
  -> commit
```

主候補は従来どおりlive GapPlannerから生成する。反対側候補はGapPlannerのsnapshotで
生成し、試算中はlive continuity stateを変更しない。反対側を実際に採用したときだけ、
選択した先頭targetをlive continuityへcommitする。

## 2. 将来リスク判定

各active horizon sampleについて、targetからlower/upper boundまでの小さい側を
`corridor_reserve`として集計する。次を将来リスクとする。

- candidateが既に不成立またはsolver backoff中
- static wall preflightがmargin escapeを使用
- tracking wall contractがactive
- horizon内の最小corridor reserveが既存runtime wall preplan reserve未満

最初にreserve不足となるpath distanceをthreat distanceとして保存する。

## 3. 反対側の予防評価と採用

主候補がhard failure、または将来リスクありの場合だけ反対側を評価する。

採用順位は次のとおり。

1. usable候補を優先
2. hard failureのない候補を優先
3. margin escape / tracking contractを必要としない候補を優先
4. corridor reserve不足のない候補を優先
5. 同tierなら既存runtime wall preplan reserve以上のreserve差がある場合だけ切替

両側が同程度なら主候補を維持し、40 Hzの左右チャタリングを作らない。

## 4. ログ

既存`Overtake decision trace: stage=planning`へ次を追加する。

- primary/alternate: `forecast`, `threat`, `min_reserve`, `threat_distance`
- decision: `alternate_trigger`, `selection_reason`, `proactive_alternate`

連続値だけの微小変動ではログを増やさず、threat分類・評価理由・選択理由の変更時に
即時出力する。既存attempt IDとmission episodeを維持する。

## 5. 非対象

- active Pass/no-return後のcross-side切替
- Recoveryアルゴリズム
- wall clearanceや追い越し速度の調整
- 全車両を含むMPCC問題への拡張
