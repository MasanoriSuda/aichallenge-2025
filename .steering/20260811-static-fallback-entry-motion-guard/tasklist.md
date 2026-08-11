# Tasklist

- [x] 20260811-171932の失敗Missionと正常Missionを比較する
- [x] static fallbackを全面禁止できないことを確認する
- [x] pure core entry-motion admission policyを追加する
- [x] candidate generatorへ接続する
- [x] dev/cloud設定と起動ログを追加する
- [x] pure core境界テストを追加する
- [x] 対象packageをbuild/testする
- [x] 動的確認項目と静的検証結果を記録する

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25 tests）
- `git diff --check`: 成功
- 初回buildではcore関数内の許容誤差定数のscope誤りを検出し、関数内定数へ修正後に
  再build/testを完了した。

## 次回実走の判定

1. 起動ログに `V2X static-fallback entry motion guard: enabled, max_shift=1.50 m` が出る。
2. `mission candidate selected`またはcandidate rejectionに
   `static_fallback_entry_motion_rejected=N` が出る。
3. 失敗地点で`lateral_shift=3.08`級のstatic fallbackを凍結しない。
4. `lateral_shift=0.68`／`1.21`級の候補は従来どおり選択・完遂できる。
5. `locked target stale or lost -> solver_unsafe -> Reverse`と67秒級外れ周が再発しない。
