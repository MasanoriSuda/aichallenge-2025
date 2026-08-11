# Requirements

## 目的

`output/20260812-081048` の4周後半で発生した逸走を防ぐ。

## 確認した事象

- WP278、実速度 8.41 m/s で `Cruise -> LowSpeedAvoidance`。
- 通常MPCが右旋回用に約 -0.30 radを出していた直後、低速車回避の直接制御が
  `Shift -> Pass`へ即時遷移した。
- 直接制御の横加速度制限が操舵全体を約 -0.06 radへ縮め、曲線追従に必要な
  操舵まで失った。
- 約1.1秒後に壁preflight不成立と緊急制動が発生し、経路から約4 m逸脱した。
- OSQP failureはなく、通常OvertakeLineのbody-clear handoffも正常だった。

## 要件

1. 低速車回避へ入る際、corridor内というだけで直接`Pass`へ入らない。
2. `Pass`移行には横位置・姿勢の収束と、locked targetの現在・予測footprint分離を要求する。
3. 低速車回避の横加速度制限は曲線追従操舵をゼロ側へ削らず、直前の追従操舵を基準に追加補正だけを制限する。
4. 壁停止、最大操舵、操舵レート、緊急制動の既存優先順位は維持する。
5. 通常OvertakeLineと評価インターフェースは変更しない。

## 変更範囲

- `multi_purpose_mpc_ros` の低速車回避直接制御
- 同coreの単体テスト
- 設定値は変更しない

## 変更禁止

- `aichallenge_system/`
- ROS topic/service/message契約
- 通常OvertakeLineの最新body-clear handoff
- ユーザー生成の`aichallenge/result-summary.json`
