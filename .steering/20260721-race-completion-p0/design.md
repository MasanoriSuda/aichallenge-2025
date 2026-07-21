# Design

## 停止車回避速度

直接操舵ownershipは既存どおり車列clearと再合流まで保持する。速度だけを次のフェーズで選ぶ。

- Shift: 回廊へ入るまで`v2x_low_speed_avoidance_shift_velocity`。
- Pass: 回廊確定後から関連車両がclearになるまで専用pass速度。
- Rejoin: clear hold成立後からMPC probe成功まで専用rejoin速度。

Passはまず6.0 m/s、Rejoinは4.0 m/sを2025 AWSIMシミュレーション向け暫定値とする。
11.1 m/sへの即時切替や車列内MPC handoffは行わない。

## Waypoint association

waypoint座標とheadingだけを受け取る純粋coreを追加する。通常時は前回IDを中心とする
前後の距離窓を検索し、以下を加重したscoreが最小の候補を採用する。

- 車両位置との距離。
- 車両yawと経路headingの差。
- 前回IDからの後退量。
- 速度と制御周期から大きく外れた前進jump。

ローカル候補の距離がlost閾値を超えた場合だけ全経路を同じheading scoreで探索する。
circular pathではID差分をwrapし、周回端の連続性を保つ。

## Recovery fault retry

既存のrecovery coreを置き換えず、セッション永久ラッチの手前にsimulation-only retry gateを置く。
設定時間、次の条件が連続成立した場合にrecovery sessionをresetし、通常のstuck detectorから再評価する。

- odometryがfreshかつfinite。
- gear reportがfreshでDrive。
- 現在footprintと短い前進／後退候補の少なくとも一方が評価可能。
- boost状態とV2X completenessが回復している。

一条件でも崩れたら確認時間をresetする。実車相当または設定無効時は従来の永久ラッチを維持する。

## 影響範囲

- `mpc_controller_cpp.cpp`: 速度フェーズ、waypoint association、fault retry adapter。
- 新規pure core: waypoint associationとfault retry timer。
- `config.yaml`: simulation-only暫定値。
- `docs/spec/mpc-integration.md`: 現行仕様。

