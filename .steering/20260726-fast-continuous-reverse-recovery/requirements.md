# Requirements

## 目的

スタック復帰時に約0.4 mごとにReverseとDriveを切り替える現行の反復を減らし、
静的掃引領域とV2X後方corridorが安全なら、一度のReverseで連続後退して速やかに
LowSpeedRejoinへ移行する。

## 根拠

対象ログは `output/20260726-062703/d1/autoware.log`。

- スタック確定から最初のReverse開始までは約0.55秒であり、gear応答は主な遅延ではない。
- 1回の後退は実移動約0.28〜0.30 mで終了し、Driveへ戻して再判定している。
- 最初のescape確定までReverse/Driveを5回反復し、約17秒を要した。
- LowSpeedRejoin timeout後に同じ短距離反復へ戻り、Normal復帰まで約44秒を要した。

## 要求

1. 現在車体が占有壁セルと重複しておらず、連続後退rolloutが静的・V2Xとも安全な場合、
   stepwise escapeを使わず一度のReverseへコミットすること。
2. 後退中も毎周期、壁、車体接触悪化、V2X corridor、gear、odometryを監視すること。
3. 車体が壁から離れ、前進rejoin rolloutが安全で、設定した最小後退距離を満たしたら、
   固定2 mを待たずに停止してDriveへ移行できること。
4. 前進経路がまだ安全でない場合は、従来の最大2 mをescape目標として継続すること。
5. 停止距離reserveを使い、目標距離付近でReverse加速から制動へ切り替えること。
6. 車体が占有セルと重複中、または連続rolloutが成立しない場合は、既存のstepwise/fail-closedを残すこと。
7. Reverse加速度指令は公式ルール目安の `1.0 m/s^2` を超えないこと。
8. 機能はsimulation-only設定で有効化し、設定で無効化できること。

## 対象外

- 実走による効果確認
- 実車向け設定
- V2X、gear、control commandのtopic契約変更
- スタック検出そのものの緩和
