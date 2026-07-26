# Task List

- [x] 最新P1ログで見かけ上のPass完了と直後の別車両接近を確認する
- [x] 要件と設計を記録する
- [x] 直前の3パラメータ実験を戻す
- [x] 全V2X車両の復帰経路占有判定を実装する
- [x] PassからのReturn開始を占有中は抑止する
- [x] Return開始直後の新規占有でPassへ戻す
- [x] 状態変化ログを追加する
- [x] 単体テストを追加して実行する
- [x] 対象パッケージをビルドする

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 182 tests、失敗なし
- 対象package test result: 618 tests、0 errors、0 failures

## 試走確認

- `OvertakeLine return corridor: Blocked` 中に `Pass -> Return` へ入らないこと
- `Released` 後に `Pass -> Return` へ進むこと
- Return開始直後に新規blockerが入った場合、
  `Return -> Pass, reason=different vehicle entered return corridor` が出ること
- 復帰待ち中の `v_limit` が5.0 m/s以下であること
- P1がロック対象を抜いた後、別車両へ0 m近傍まで接近せず走行を継続すること
