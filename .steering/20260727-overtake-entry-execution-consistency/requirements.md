# Requirements

## 目的

`make dev2` の低速テンプレ車追い越しで、候補選択時には通過可能と判定した
ラインが、`ShiftOut` 開始直後または実行中に静的壁・横加速度条件で
`Recovery` へ落ちる不整合を減らす。

## 変更範囲

- `multi_purpose_mpc_ros` の追い越しライン入口判定
- 実行時と入口で共用する静的壁・横加速度 horizon 評価
- 幾何的に失敗した同一対象・同一側への短時間の再試行抑止
- 既存の pure helper / controller 単体テストによる回帰確認
- 参加者向け MPC 暫定仕様

## 制約

- `v2x_prediction_use_course_lateral_velocity: false` を維持する。
- 実壁接触、前方Emergency、他車による明確な閉塞のguardは緩和しない。
- 速度cap、車体寸法、壁余裕、横加速度の設定値は今回変更しない。
- start-grid専用の検証済み車間corridorは既存契約を優先する。
- ROS topic/service、Domain、評価成果物の契約を変更しない。

## 完了条件

- 通常追い越しの新規`ShiftOut`前に、実行時と同じ静的壁・横加速度評価が走る。
- 入口不成立時は`ShiftOut -> Recovery`を発生させず、候補側を棄却する。
- 実行中の幾何失敗後、同一対象・同一側へ即時再突入しない。
- 既存の追い越しcoreテストと対象packageのビルドが通る。
- 実走効果はユーザーの`make dev2`で確認する。
