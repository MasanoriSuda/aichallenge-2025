# Design

## 方針

採用済み軌道はmove先の`validated_horizon`が所有しているため、
弾性ターゲットクリアランスの判定も同オブジェクトを参照する。

```text
candidate_horizon
  -> move -> validated_horizon
                  -> target_eyを参照
```

これによりmove済みvectorの添字参照を除去する。計算順序、閾値、
速度制限、壁・車体クリアランスは変更しない。

## 影響範囲

- `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- 弾性クリアランス有効時の診断フラグ判定のみ

