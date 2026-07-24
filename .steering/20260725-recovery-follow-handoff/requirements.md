# Overtake Recovery追従引継ぎ実験 要件

## 目的

移動中の先行車に対する追い越し失敗後、OvertakeLine Recoveryの固定速度上限が
通常Followの距離制御へ追加されることで車間が開き、再接近と再試行を繰り返す負の
ループを弱める。

Recovery中も横方向は基準線へ安全に復帰しつつ、縦方向は有効な移動先行車に対する
通常Followと同じ距離連動速度上限を使用する。

## ベースライン

- run: `output/20260724-235653`
- d2 `domain_v_max`: 16.0 km/h
- d1 Lap 1 / 2: 75.193 / 74.948 s
- d2 Lap 1 / 2: 73.694 / 75.023 s
- d1 OvertakeLine Recovery: bag全体57回
- d1/d2とも5トピックMCAP取得済み

## 変更範囲

- `multi_purpose_mpc_ros`のOvertakeLine Recovery速度上限選択
- 上記選択ロジックのcore単体テスト
- 既存周期debugへの速度モード1項目追加

設定値、topic/service/message、Domain、評価schemaは変更しない。

## 維持する安全条件

- 壁実接触時の0 m/s上限
- 静的壁、wall margin、横加速度による経路制約
- EmergencyBrake、front risk、deceleration guard
- solver Recovery時の固定Recovery上限
- 先行車ロスト、不連続、停止時の固定Recovery上限
- Recoveryの距離、横偏差、stall、timeout終了条件

## 実験条件

- `make autoware-build`と対象coreテストが成功すること
- dev2、d1/d2、2周以上
- d2は16 km/h、rosbagは現行5トピックを維持
- 現行run `20260724-235653`と同じ指標で比較する

## 観測項目

1. Recovery回数、理由、継続時間
2. Recovery中のfixed/follow速度モードと実効上限
3. Recovery開始後のd1速度低下量とd1-d2速度差
4. Recovery後の車間拡大量と再ShiftOutまでの時間
5. OvertakeのShiftOut、Pass、Return完了数
6. SafetyStop、wall contact、Reverse、solver異常
7. lap time、MCAP topic数・rate・最大gap

## 当初の採否基準

- 採用候補:
  - Recovery後の引き離され量または低速時間が減る
  - Recovery再試行ループが悪化しない
  - 接触、EmergencyBrake、solver異常が増えない
- 不採用:
  - 車間過小、接触、停止先行車への加速が発生する
  - Recovery回数またはsolver失敗が明確に悪化する
  - 追従引継ぎが有効にならず現行と同等である

## 暫定採用判断

当初基準ではSafeStopにより不採用となるが、映像とログを再照合すると、走行中止は
Follow引継ぎそのものよりhairpin内側Pass後の相手車両との進路交差が主因と考えられる。

本ソフトは競技シミュレーション用であり、現段階は性能限界と課題を早期に出すことを
優先する。そのため、Recovery Follow引継ぎは暫定採用し、hairpinのpass side選択と
復帰条件を残課題として別途扱う。

この暫定判断は実車適用を許可するものではない。
