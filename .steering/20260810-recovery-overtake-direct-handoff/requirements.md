# Requirements

## 目的

事故・停止車を契機に Stuck Recovery が始まった後、完全な前進追い越し Mission が
成立しても Reverse / ForwardCreep / LowSpeedRejoin を最後まで実行する現象を解消する。

## 要件

- 開始済みの協調停止 Recovery から、検証済み Overtake へ早期に所有権を返せる。
- Reverse 中は即座に前進へ切り替えず、停止してから Drive を要求する。
- fresh な Drive report と非後退速度を確認してから Recovery を解除する。
- 壁接触、solver failure、衝突悪化、reverse-only episode では解除しない。
- V2X / 評価インターフェース、既存パラメータ、実車構成は変更しない。
- `aichallenge/result-summary.json` の既存変更には触れない。

## 非対象

- 通常 Overtake の左右選択・Mission rollout
- Reverse 距離・速度の再調整
- 接触を許容する条件の追加

