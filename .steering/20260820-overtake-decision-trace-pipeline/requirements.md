# Requirements

## 目的

現行の追い越し機能を凍結したまま、1回の制御周期について、戦術判断から
`/control/command/control_cmd`へpublishされた指令までを同じdecision IDで追跡可能にする。

## 変更範囲

- `multi_purpose_mpc_ros`内の追い越しexecution authority診断
- MPC solve結果、post-process、Recovery/failsafe、最終ROS出力の診断
- 既存のchange-awareログと単体テスト

## 制約

- 追い越し戦術、速度制限、壁判定、Recovery判定、MPC重み、設定値を変更しない
- `/control/command/control_cmd`の名前・型・publish経路を変更しない
- 40 Hzで毎周期ログを出さず、状態変化・異常・heartbeatに限定する
- `output/`、result JSON、クラッシュダンプへ変更を加えない

## Definition of Done

- 最終controlログ一行にauthority判断と同じdecision IDが出る
- 最終controlログから、phase、target、path source、速度owner、solver/fallback、
  Recovery、実際の速度・加速度・操舵指令を一行で確認できる
- dynamic-obstacle escapeがGapPlannerの生成結果を利用する正常系を
  multiple-authority競合として報告しない
- 対象単体テストと`make autoware-build`が成功する
