# Tasklist

- [x] 最新P1ログから2周目の左右候補と棄却順序を確認
- [x] 要件と設計を記録
- [x] inner curve precommit判定をcore helperとして追加
- [x] controller設定読込と判定へ統合
- [x] competition configで限定的に有効化
- [x] 正常系・安全棄却系の単体テストを追加
- [x] 対象packageをビルド
- [x] 単体テストを実行
- [x] ユーザー実走確認項目を記録

## Definition of Done

- 内側corridorが有効で相対速度が設定下限以上なら、完了距離不足だけを理由に棄却しない。
- corridor不足、緊急制動、前方距離不足、相対速度下限未満ではprecommitしない。
- 既存のV2X overtake coreテストが通る。
- `multi_purpose_mpc_ros` がビルドできる。

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 624 tests、0 errors、0 failures、0 skipped

## 実走確認項目

1. 起動時に `V2X inner curve precommit: enabled` が出ること。
2. 2周目以降のヘアピン前で、内側corridorが有効な時点に
   `V2X behavior: Follow -> Overtake` が出ること。
3. 続けて `OvertakeLine: Idle -> ShiftOut, side=-1` が出ること。
4. 従来の `curve entry lacks measured gain/near-target range` だけで内側候補を失わないこと。
5. corridor不足・壁余裕違反・横加速度超過・緊急制動では従来どおり進入しないこと。
