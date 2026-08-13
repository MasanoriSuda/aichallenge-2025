# Tasklist

- [x] 20260813-125441 と現行 HEAD の failure 経路を照合する
- [x] requirements / design を作成する
- [x] hard / soft fallback を実装する
- [x] progress-aligned warm start を実装する
- [x] shadow hold / last-feasible scope を修正する
- [x] shadow timing log を追加する
- [x] unit test を追加する
- [x] package test / build を実行する
- [x] 差分と実走確認項目を整理する

## 実走で確認する項目

- `target-side separation conflicts...` の直後に旧 horizon を実行せず phase rebuild へ移ること
- soft optimizer failure では同一 phase / generation / side の last-feasible のみ保持すること
- `source=last_feasible` が phase 遷移後に残らないこと
- active Pass の `CurrentSideHold` が `progressive_entry_incomplete` にならないこと
- `timing_ms=total/left/right/resolve` と `/control/command/control_cmd` 間隔の相関
