# Start Grid V2X Time / Recovery Guard Requirements

作成日: 2026-07-28
状態: Complete

## 背景

提出環境ログ `/output/20260728-030217/d1/autoware.log` では、3台構成のV2X配列が
`vehicles=2, message_vehicles=2`で正常に届いていた。一方、制御周期が保持するROS時刻より
同時実行中のV2X callback受信時刻が最大約35 ms新しくなる周期があり、
`receipt_age=-0.035`として全V2X車両が一時的に無効化された。

この瞬断でStart-grid中のBehaviorが`Cruise / Overtake`と`SafetyBrake`を往復し、
静止中の前方グリッド車を`coordinated_stop`として扱った。壁・衝突証拠がないまま
`coordinated_stop_qualified=1`となり、P1が発進前にReverse Recoveryへ入った。

## 要求

1. V2X callbackと制御周期の同時実行による小さな負のreceipt ageをfreshとして扱う。
2. future toleranceを超える受信時刻、timeout超過、非有限値は従来どおりrejectする。
3. Start-grid grace、動的観測、または選択済みbreakout中は、新規coordinated-stop Recoveryを開始しない。
4. Start-grid以外で停止車列に巻き込まれた場合の既存coordinated-stop Recoveryは維持する。
5. 既存のV2X台数完全性、ID、sample、static/V2X corridor、gear安全条件を緩和しない。
6. ユーザーの暫定本番用`config.yaml`変更を変更・巻き戻ししない。

## 変更範囲

- `start_grid_grace` pure helperと単体テスト
- `v2x_overtake_core` receipt-age pure helperと単体テスト
- `mpc_controller_cpp`のV2X freshness判定とcoordinated-stop入口
- `docs/spec/mpc-integration.md`

## Definition of Done

- 約-35 msのreceipt ageがfreshとなる。
- -50 msの許容境界を超えるfuture receiptはrejectされる。
- Start-grid中のcoordinated-stop候補が抑止される。
- Start-grid終了後のcoordinated-stop候補は従来どおり成立する。
- 対象単体テストと`make autoware-build`が成功する。
- ローカル3台実走を行えない場合は、未検証事項として明記する。
