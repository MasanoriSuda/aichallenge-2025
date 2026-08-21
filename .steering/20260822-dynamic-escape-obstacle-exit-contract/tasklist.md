# Tasklist

- [x] 最新走行ログと壁handoffログを照合する
- [x] 壁拒否後に横方向の実行権を失う経路を特定する
- [x] `DynamicEscapeExitGate` と理由enumを追加する
- [x] 壁拒否周期にも直前の物理検証済み解を復元する
- [x] 前方障害物未解消時に終了を保留し、再計画を要求する
- [x] 状態変化ログを追加する
- [x] gateの単体テストを追加する
- [x] package build / testを実行する
- [x] 差分をレビューしてコミットする

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_overtake_execution_orchestrator`: 43 tests passed
- 追加した終了契約テスト: 7 tests passed
- `git diff --check`: 成功

## 試走時の確認

- `Dynamic escape exit contract: event=entered` 後、前方車未解消なら
  `reason=target-blocking` となること
- 壁拒否直後に `retained=1` で横操舵が短時間維持されること
- 別解が壁判定を通ったら `event=replacement-adopted` で終了すること
- RacingLine / SafetyBrake と Dynamic Escape の周期的な往復が減ること
- wall contact、solver fallback連続数、停止時間が増えていないこと
