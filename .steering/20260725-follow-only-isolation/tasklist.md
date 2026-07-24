# 5 m追従単独分離実験 タスクリスト

## 準備

- [x] 16 km/h基準run `20260724-235653`を特定する
- [x] d1/d2の2周と5トピックMCAPを確認する
- [x] Followを維持するOvertake抑止方法をコードで確認する
- [x] requirements / designを作成する

## 実験設定

- [x] start-grid breakoutを一時無効化する
- [x] 通常Overtakeの必要通路幅を一時的に100 mへ変更する
- [x] Follow、速度、安全設定が不変であることを確認する
- [x] `git diff --check`
- [x] `make autoware-build`

## 動的検証

- [x] dev2を起動する
- [x] d1/d2を2周以上走行する
- [x] d1/d2の5トピックMCAPを確認する
- [x] Overtake / Recoveryが0回であることを確認する
- [x] 5 m境界のcommand / actual / accelerationを抽出する
- [x] 車間、follow_cap出入り、急減速回数を基準runと比較する
- [x] SafetyStop / contact / Reverseとlap timeを確認する

## 完了

- [x] `results.md`へ原因判定を記録する
- [x] 実験用2設定を元へ戻す
- [x] 最終差分に実験設定が残っていないことを確認する
