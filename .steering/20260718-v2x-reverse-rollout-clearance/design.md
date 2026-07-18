# Design

## 方針

従来判定は、選択ロールアウトの横方向包絡と最大後退距離から1個の矩形を作り、
他車の現在位置と予測位置を膨張円として重ねていた。これは安全側だが、後退と逆方向にある
前方車まで障害物として固定し、停止列の最後尾を決められない。

pure C++ の `recovery_footprint` に、円形V2X障害物と復帰ロールアウトの
クリアランス評価を追加する。各ロールアウト姿勢で、非対称な向き付き車体矩形と他車円の
signed clearanceを求める。

## 判定規則

- 初期clearanceが非負:
  - 全姿勢で非負ならclear。
  - 途中で負になれば `NewOverlap` としてblock。
- 初期clearanceが負:
  - 保守的な膨張により初期から重なっている状態として扱う。
  - 各姿勢でclearanceが単調非減少であることを必須とする。
  - 最終clearanceが初期値から有意に改善しなければblock。
  - これにより、前方車から離れる最後尾の後退だけが進み、後方車へ近づく前方車は待つ。

V2X速度が停止判定上限以下の場合だけこの分離判定を使う。移動車には従来の
現在位置から予測位置までを含む矩形回廊を使用する。

## ROS adapter統合

静的mapで選ばれた `FeasibilityResult::rollout` をV2X判定まで保持する。
停止車ごとにpure coreを呼び、最初のblockerを `RecoverySafetySnapshot` に保存する。
forward deadlock fallbackも、そのforward rollout自身で再評価する。

## 安全性

- map footprint判定とgear report確認は変更しない。
- 初期V2X重なりからの逃避は、全サンプルで悪化しない場合だけ許可する。
- stepwise escapeは0.4 mごとに停止・再評価する既存上限を維持する。
- 不完全・不正V2X情報は従来どおり後退を許可しない。
- 実車は既存の `simulation_only` gateで対象外のままとする。

## 検証

`test_recovery_footprint` に以下を追加する。

1. 前方円との初期重なりが後退で減少する。
2. 再現ログ相当の前方・右寄り円から選択Right rolloutで分離する。
3. 後方円との初期重なりが後退で悪化する。
4. 初期clearから旋回途中で円へ接触する。
5. 不正入力をfail closedにする。
