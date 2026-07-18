# Extended Recovery Budget and Rejoin Window Design

作成日: 2026-07-18
状態: Completed (Rejected)

## 方針

今回は状態機械や安全判定を変更せず、`config.yaml`のsimulation-only予算を変更する。

- Side / Mixed contactは既存の適応操舵、contact減少最大選択、選択角latchを使う。
- 11〜16 stepも各step開始前に同じstatic / V2X候補選択を行う。
- 予測または実測contact悪化、Unknown、距離・速度・時間上限で従来どおり停止する。
- LowSpeedRejoinは既存の速度cap、MPC history reset、前方swept footprint、solver graceを使う。
- timeoutだけを10秒へ延長し、許容横偏差やheading閾値は緩めない。

## 判定

- D3が追加stepでcontact clearまたはepisode 2.0 mを達成すればstep予算仮説をPassとする。
- D3のcontact改善が止まる、または16 stepでも未達なら、step増加ではなく通常MPCのwall侵入抑制を次課題とする。
- D2が閾値内へ収束して`rejoin_complete`すればtimeout仮説をPassとする。
- 10秒でも横偏差が収束しない場合は、時間延長ではなくLowSpeedRejoinの横経路制御を修正する。

## 安全性

独立した3.0 m距離上限と全hard gateを維持するため、step回数・再合流時間の延長だけで
未検証候補を駆動しない。実験終了後は全コンテナを停止する。
