# Requirements

## 目的

GapPlanner と壁 preflight を通過した動的障害物回避候補について、最初の実追従 QP が成功する前に実行可能な branch として扱わない。資格確認 QP が失敗した場合は、減速 fallback へ遷移せず、直前の実行可能な通常走行指令を保持して候補を隔離する。

## 制約

- 追い越し、壁余裕、OSQP の設定値は変更しない。
- 成功候補に追加の QP solve を課さず、最初の実追従 QP 自体を資格確認として使う。
- EmergencyBrake、既に資格確認済みの実行 branch、通常の solver failure の挙動は変更しない。
- ROS 2 topic、message、service、launch、提出物契約を変更しない。
- `aichallenge/result-summary.json` の既存変更は対象外とする。

## 完了条件

- 未資格候補は planning trace で `qualification-pending-*` と識別できる。
- 初回実追従 QP 成功時に `qualified` trace が出る。
- 初回実追従 QP 失敗時に `qualification-rejected` trace が出る。
- 資格確認失敗時は branch を隔離し、直前の有限な通常走行指令を保持する。
- 保持できない場合だけ従来の安全 fallback を使う。
- 関連単体テストと package build が成功する。
