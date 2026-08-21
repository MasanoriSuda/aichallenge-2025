# Requirements

## 目的

直近走行 `output/20260822-031809` で頻発した Dynamic Escape の
tracking qualification QP失敗を、MPC/MPCC接続と横プロファイル連続性の
不整合として修正する。

## 観測事実

- qualification rejectは48件で、全件がOSQPの
  `maximum iterations reached`。infeasible判定ではない。
- 47件はwarm-start失敗後のcold retryでも収束しなかった。
- corridor幅より`target_adjust`との相関が強く、0.5--1.0 mでは
  qualification rejectが21/22件だった。
- Dynamic EscapeはOvertakeLine phaseがIdleのため、現行の
  `progress_contouring_mpcc_overtake_only`ではlegacy MPCへ残る。

## 要求

1. Dynamic Escapeをcontouring-progress MPCCの実行対象に含める。
2. 横到達性を各stageの現在状態基準の独立判定ではなく、前stageの
   選択横位置・横速度から連続して判定する。
3. 既存の壁・車体collision corridorを緩和しない。
4. MPCC preparation/extended solverが不成立な場合の既存縮退を維持する。
5. qualificationログから、solver formulation、mode reset、連続プロファイルの
   最大横移動・必要横加速度を確認できるようにする。
6. ROS topic/service、提出・評価インターフェースを変更しない。

## 変更範囲

- `multi_purpose_mpc_ros` のcore helper、controller、trace、単体テスト
- `docs/spec/mpc-integration.md`
- `config.yaml`の既定値変更は行わない。

## Definition of Done

- Dynamic Escapeでprogress-contouringがrequestされる単体テストが通る。
- 連続横profileが、独立stageでは見逃す不連続をrejectする単体テストが通る。
- tracking traceに新しい診断項目が出る。
- 対象packageがbuild/testを通る。
