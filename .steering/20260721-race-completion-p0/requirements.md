# Requirements

## 目的

ChatGPT Proレビューで確認された、6周レースの完走率と追い越し後の速度回復へ直接影響する
P0問題を修正する。

## 要件

- 停止車回避の直接操舵を、横移動、車列通過、再合流の速度フェーズへ分ける。
- 車列通過中はMPCへ無条件に戻さず、既存の直接操舵ownershipを維持する。
- 通常の自己位置対応付けは前回waypoint近傍を優先し、距離、heading、到達可能進捗で選ぶ。
- 明示的な初期化、経路更新、またはローカル探索から明確にlostした場合だけ全経路探索を許可する。
- 復帰中の一時faultは、シミュレーション限定設定が有効で、odom、gear、指令、短距離回廊が
  継続して正常な場合だけ再試行可能にする。
- topic、service、message型、Domain、評価JSONの契約は変更しない。
- OSQP workspace再利用と周回別race strategyは本変更に含めない。

## Definition of Done

- 直接操舵のShift、Pass、Rejoin速度選択を純粋関数テストで確認する。
- ヘアピンの近接する別枝より、前回進捗とheadingが連続するwaypointを選ぶテストが通る。
- lost時のみ全経路へ再捕捉できるテストが通る。
- fault再試行の正常継続時間、異常reset、simulation-only gateをテストする。
- 対象パッケージのテストとビルドを実行する。

