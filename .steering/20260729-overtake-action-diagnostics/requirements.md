# Overtake Action 診断ログ要件

## 目的

通常 Overtake の失敗・中断時に、どの transition action が他の候補より優先されたかを
一行で特定できるようにする。

## 要件

- Action 発生時に、Action名、phase、target、mission/Behavior sideを記録する。
- wall、return corridor、rear-clear、side replan、progress watchdog の入力事実を記録する。
- 同じActionが継続する間は一度だけ記録する。
- `None`へ戻った後に同じActionが再発した場合は、新しいイベントとして記録する。
- 既存のphase遷移ログとreason文字列は維持する。
- 制御判断、設定値、ROS 2インターフェースは変更しない。

## Definition of Done

- Action名とログ重複抑止が純粋関数としてテストされる。
- `make autoware-build`が成功する。
- `multi_purpose_mpc_ros`のテストが成功する。
