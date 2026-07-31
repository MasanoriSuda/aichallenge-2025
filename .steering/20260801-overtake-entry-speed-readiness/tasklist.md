# Tasklist

- [x] 最新ログとrosbagから再現値を確定
- [x] 要件・設計を記録
- [x] entry speed readiness pure helperを追加
- [x] 前方・側方target速度を共通化
- [x] 新規Overtake最終ゲートを追加
- [x] 設定値と起動ログを追加
- [x] 再現ケースを含む単体テストを追加
- [x] `make autoware-build`を実行（25 packages成功）
- [x] packageテストを実行（721 tests、失敗0）
- [x] `git diff --check`を実行

## Dynamic verification

未実施。以下は利用者側の`make dev2`試走で確認する。

- `20260801-074818/d1`相当の`ego-target <= -1.7 m/s`で`Idle -> ShiftOut`が発生しない。
- 遮断中は`desired=Overtake, final=Follow`と`overtake entry speed not ready`が出る。
- 相対速度が-0.5 m/s以上になってから0.3秒未満では開始しない。
- 0.3秒連続成立後は、その他の既存条件が成立していれば新規Overtakeを開始できる。
- start-grid発進、既存ShiftOut/Pass、FollowPrepare再開を遮断しない。
- 壁Recovery、接触、完遂数は別途比較し、この修正だけで安全問題全体が解消したとは判定しない。
