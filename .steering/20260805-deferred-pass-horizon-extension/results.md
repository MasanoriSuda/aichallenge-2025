# 実装結果

## 変更

- rear-clear予測がcommitted static horizon外かどうかを判定する純粋関数を追加した。
- static horizon残距離が`v2x_overtake_pass_horizon_revalidation_lead_distance`
  （現行3.0 m）より長い間は、rear-clearだけを理由とするPass extensionを延期する。
- 残距離3.0 m境界へ到達すると従来どおりextension/refreshを要求する。
- 延期待ち中もrolling outer replanを通常Pass horizon処理より先に評価する。
- 1 Passにつき1回だけ`OvertakeLine Pass rear-clear extension deferred`を記録する。

## 前回ログへの適用

`20260805-001505/d1`の代表値を回帰試験に使用した。

```text
required rear clear: 24.3 m
committed static:    21.6 m
lead:                 3.0 m
```

旧判定ではPass開始直後にextensionした。新判定では残り21.6 mで継続し、残り3.01 mでも
待ち、3.00 m到達時に初めてextensionを要求する。

## 維持したhard trigger

- predicted footprint overlap
- dynamic prediction expiry
- static/dynamic horizon lead到達
- absolute Pass time/distance limit
- wall/contact/Emergency/target continuity

## 検証

- `make autoware-build`: 成功
- package test: 25/25 targets成功
- 843 tests、0 errors、0 failures、0 skipped
- overtake core: 346 tests、0 failures
- `git diff --check`: 成功

動的走行は未実施。次走ではPass開始直後のSafeSeparation減少、deferredログ、rolling outer
accept、`Pass -> Return` / `Pass -> Recovery`比率を確認する。
