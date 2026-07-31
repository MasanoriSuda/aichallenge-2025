# 要件

## 目的

通常Overtakeを、固定量の横移動ではなく、予測区間で必要な最小横移動だけで完遂する。

## 必須挙動

1. 車体・予測余裕・壁余裕を反映した候補区間に基準レーシングライン（`e_y = 0`）と現在位置が含まれる場合、横ShiftOutを行わず同じラインでPassへ入る。
2. 基準ラインが候補区間外の場合、候補区間内で基準ラインに最も近い点を横目標にする。
3. 左右とも実行可能であれば、基準ラインを維持できる側、次に必要横移動が小さい側を優先する。
4. 左右とも安全な候補がなければpass sideを選ばず、Overtakeや横prepositionへ入らずFollowを継続する。
5. 実行開始後の物理接触、壁余裕違反、solver異常に対するRecoveryは無効化しない。

## 制約

- ROS topic/service/message契約は変更しない。
- start-grid専用corridorと停止車両用LowSpeedAvoidanceは今回の優先順位変更の対象外とする。
- 既存のCandidate B（ShiftOut相対速度0.8～2.0 m/s）は維持する。
- 設定で従来の経路選択へ戻せるようにする。

## Definition of Done

- 純粋関数の単体テストで基準ライン維持、最小横移動、候補なしを確認する。
- 通常Overtake開始時に選択方式と横目標をログで識別できる。
- 対象packageのテストと`make autoware-build`が成功する。
