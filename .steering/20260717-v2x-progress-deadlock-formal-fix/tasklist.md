# V2X共通進捗・停止デッドロック正式修正 Tasklist

作成日: 2026-07-17
状態: Complete（runtime制約を記録）

## Definition of Done

- [x] 共通reference path進捗で前方車を早期検出する。
- [x] circular終端、後方車、横範囲、逆向きsegmentをunit testする。
- [x] SIMのP1/P2/P3で安全条件付きstuck recoveryを有効化する。
- [x] SafetyBrake/EmergencyBrakeの優先順位を維持する。
- [x] debug logへ共通進捗の診断値を追加する。
- [x] 対象testとbuildが成功する。
- [x] dev3で前走車追突による3台停止列の再発有無を確認し、個別wall SafeStop制約を記録する。

## Tasks

- [x] baseline `output/20260717-082215`を確認
- [x] requirements/design/tasklistを作成
- [x] pure共通進捗射影helperを実装
- [x] MPC V2X behaviorへ統合
- [x] config parsing / validation / logを追加
- [x] P1/P2/P3のSIM recoveryを有効化
- [x] V2X source/receipt clock-domain判定を分離
- [x] circular閉路点・連続重複点を共通進捗で安全にskip
- [x] stepwise停止・再評価とForward Left/Right fallbackを実装
- [x] unit testを追加
- [x] `test_v2x_overtake_core`を含む対象3 suiteを実行
- [x] `make autoware-build`
- [x] `make dev3` runtime確認
- [x] `docs/spec/mpc-integration.md`を更新
- [x] `git diff --check`
