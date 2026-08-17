# Design

## 問題1: optimizerと実行物理制約の差

従来のoptimizerは、trajectory由来の左右境界とwall reserveをbox制約として使う。
一方、事後検証は `d'(s)` から求めたpath headingで2.0 m x 1.45 mの車体矩形を回転し、
static occupancy gridへ照合する。このためbox内の解でも、傾いた前後端が壁へ入って
事後検証で破棄され得る。

### 対策

- 物理検証済みbaseline profileの各stage headingを求める。
- baseline近傍のtrust regionだけを対象に、heading-aware footprintで連結free intervalを
  求める。
- preferred wall reserveとhard wall reserveのintervalを別々に保持する。
- optimizer request、post-validation execution bounds、下流stage MPC wall boundsへ
  同じintervalを渡す。
- profile全体のcoupled validationは最終guardとして維持する。

全コース幅を毎周期探索せず、既存の最大reference調整幅の近傍だけを調べるため、
40 Hz制御に対する探索量を限定する。

## 問題2: Recovery保持Missionの再突入

実測poseのwall hard faultはFollowPrepare中に検出されていたが、将来horizonの
physical infeasibilityは後段で判定される。そのためRecoveryRetentionのMissionが同じ
将来経路で再びRecoveryへ入り、位置が少し変わるまで往復していた。

### 対策

最終execution horizon failure処理でもRecoveryRetentionを確認する。hard horizonが
残る場合はfailed sideを短時間blockし、Mission retentionを禁止してIdleへ戻す。
実接触・Emergency・solver recoveryの既存優先順位は変更しない。

## 非対象

- wall clearanceパラメータの攻撃化
- 接触guardの無効化
- Reverse/スタックRecoveryの調整
- Pro案の左右branch costや縦横同時最適化の次段階
