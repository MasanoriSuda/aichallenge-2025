# Results

## 実装結果

Pass由来の同側rolling replanへ現在横位置を追加し、現在位置から0.35 mを超えるgoalを
entry preflight前に除外した。preflightがgoalを補正した場合も、補正後のgoalを再度同じ
admissionへ通すため、検証経路と実行経路の上限は一致する。

設定値は次のとおり。

```yaml
v2x_overtake_mpcc_lite_same_side_max_lateral_adjustment: 0.35
```

このguardはPass由来の同側rolling replanだけへ作用する。新規entry、通常の左右比較、
cross-side replanには作用しない。

## 静的・単体検証

- 直近ログ相当の `1.92 -> 0.45 m`（1.47 m補正）を棄却。
- `1.92 -> 1.70 m`（0.22 m補正）を許可。
- 負側の対称ケースも許可。
- guard無効時の従来挙動と不正入力のfail-closedを確認。
- ビルドとpackage全テストが成功。

## 動的検証待ち

試走では成功率だけでなく、`dy`、`rolling_lateral_rejected`、MPCC-lite branch計算時間、
wall Recovery、target-bound overlapを同時に確認する。大横移動が消えても候補なし待機が
増える場合は、上限を広げる前にcurrent-lineのcorridor生成と縦進捗制約を見直す。
