# Requirements

## Purpose

`make dev3`のスタートで、P1が隣グリッドのP2を前方衝突車として扱い、
P2/P3より約3秒遅れて発進する問題を解消する。

## Scope

- Start grid graceが有効な間のV2X前方車横判定
- 通常走行とヘアピン中の拡大横判定は維持する
- Gate2の停止車列回避ロジックは変更しない

## Acceptance criteria

- Start grid grace中は通常の車両衝突幅を使い、curve用横marginを加算しない
- Start grid grace終了後は従来のcurve用横marginを使う
- pure core単体テストと`make autoware-build`が成功する
- `make dev3`でP1/P2/P3の初動速度発生時刻差が1秒以内になる

