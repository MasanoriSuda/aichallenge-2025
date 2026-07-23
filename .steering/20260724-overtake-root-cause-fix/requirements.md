# オーバーテイク根本原因修正 要件

## 目的

高速設定車両が移動中の低速設定車両を追い越すシナリオで、追い越し側が
対象車より遅い速度制約を受けたまま `Pass` を保持し、対象を見失って
`Recovery` へ落ちる現象を解消する。

## 対象

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
  - V2X 追い越しの速度仲裁
  - Ready から Start へのセッション継続
  - 追い越し入口と完遂距離判定
  - 上記の単体テスト
- 必要な場合の `docs/spec/mpc-integration.md`

## 対象外

- ROS 2 topic / service / message 型の変更
- Domain 0 / Domain 1..N の責務変更
- `aichallenge_system` の評価 FSM や result JSON schema の変更
- 車両の最高速度、加速度、操舵、壁クリアランス値のチューニング
- 実車向け設定
- `output/`、rosbag、既存評価成果物の編集

## 機能要件

1. Ready 中に確立した V2X 車両速度・位置履歴を、同一セッションの
   `/awsim/state=Start` で消去しない。
2. 通常 Follow の車間回復制約を維持しつつ、確立済み ShiftOut が
   対象車速度未満の速度上限によって対象から離されないようにする。
3. close-follow は通常の新規進入距離を下回って開始せず、完遂距離は
   実際の ShiftOut / Pass 速度上限で評価する。
4. 完遂距離不足を曲線進入例外で迂回する場合は、対象が通常入口距離、
   ShiftOut距離、rear-clear距離の和以内にあり、実測で高速車が前車より
   設定速度差以上速いことを要求する。通常入口には一律の実測速度差を要求しない。
5. EmergencyBrake、位置ジャンプ、solver failure、明示禁止 waypoint、
   wall bound など既存の fail-safe を弱めない。
6. `/control/command/control_cmd` を含む既存インターフェースを変更しない。

## 再現ログ

- `output/20260724-000812/d1/autoware.log`
  - `Cruise -> Overtake`: line 147
  - Overtake 中 `desired_v=3.21`, `limit=2.11`: line 150
  - `ShiftOut -> Pass`: line 161
  - Start 後の `NoData` と start-grid breakout: lines 162-170
  - `fd=17.96`, `rel=-2.82`: line 190
  - `Pass -> Recovery`, course progress discontinuity: line 205

## 受入条件

- 追い越し速度仲裁、完遂距離、曲線進入例外を純粋関数の単体テストで再現できる。
- Start の同一セッション遷移で V2X tracking を保持するコード経路を確認できる。
- `test_v2x_overtake_core` を含む対象パッケージのテストが通る。
- `multi_purpose_mpc_ros` がビルドできる。
- topic / service / message / launch entry の契約差分がない。
- 可能な環境では `dev2` で、負の実測速度差または遠方targetへ早期進入せず、
  近距離かつ正の速度差で`ShiftOut -> Pass -> Return`とlocked targetの後方通過が
  成立することを確認する。
