# Requirements

## Purpose

現行の追い越し処理は、Gap Planner、動的回避bridge、solver backoff、反対側再評価、
authority、tracking MPCが別々に採否を決めるため、走行ログから最終棄却者を一意に
追跡できない。性能や安全閾値を変更せず、追い越し候補の意思決定を保守可能な形で
可視化する。

## Scope

- `multi_purpose_mpc_ros` の参加者実装内だけを変更する。
- 追い越し候補の各段階を共通のDecision Traceへ正規化する。
- 反対側候補の不採用理由を、planner / bridge / backoff / side mismatchに分類する。
- authority不採用理由を列挙型で保持する。
- 状態変化時と低頻度の再通知時だけログを出す。
- tracking solverの失敗・復帰を同じログ接頭辞で関連付ける。

## Constraints

- 速度、操舵、壁余裕、追い越し採用条件、FSM遷移を変更しない。
- ROS 2 topic、message、service、launch、評価成果物契約を変更しない。
- 40 Hz周期で同じ内容を出力し続けない。
- `output/`、rosbag、ユーザーの`aichallenge/result-summary.json`変更を編集・コミットしない。

## Definition of Done

- 1行のDecision Traceからprimaryとalternateの採否理由、最終authorityを識別できる。
- 連続値だけの変化ではログが増えず、分類の変化では即時にログが出る。
- authorityの各拒否ゲートとtrace分類に単体テストがある。
- 対象packageがビルド・テストを通る。
