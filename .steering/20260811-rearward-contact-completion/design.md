# Design

## 原因

`RecoverableSideContact` は接触開始から一律0.8秒で終了する。終了時点でtargetが
後方へ進み始めていても、車体矩形がまだ非重複でなければ `pass_short_horizon_safe`
がfalseになる。その結果、既存のrear-clear完遂処理へ到達する直前に
SafeSeparationがRecoveryを選ぶ。

## 方針

通常の接触許容0.8秒とは別に、rearward completion tailを設ける。

次をすべて満たす場合だけ、接触継続時間を2.5秒まで延長する。

- PassがSideBySide committed
- forward-completion latch済み
- target中心が自車より後方
- target ID／course progressが連続
- targetがpass側と反対にあり、正面衝突ではない
- 前方への実測進捗がfresh
- 既存の相対速度、相対横速度、自車速度条件を満たす

延長中も `recoverable_side_contact_active` を維持することで、既存のPass速度所有権と
SafeSeparation前進処理を再利用する。車体が非重複になれば、既存の
side-by-side rear-clear tailへ自然に引き継ぐ。

## 非対象

- `target_s < 0`だけを根拠にした即Return
- wall／EmergencyBrake／solver recoveryの緩和
- full-Mission entry admissionの段階化
- stuck recoveryの長時間反復対策

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: rearward contact completion policy
- `mpc_controller_cpp.cpp`: latch/stage/configの接続と起動ログ
- `config.yaml`, `config_for_cloud.yaml`: 専用2.5秒上限
- `test/test_v2x_overtake_core.cpp`: 許可／拒否境界テスト
- 外部interface、launch、message型: 変更なし
