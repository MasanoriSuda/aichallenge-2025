# Overtake Recovery追従引継ぎ実験 タスクリスト

## 調査・設計

- [x] Recovery固定上限とBehavior Follow上限の合成順を確認する
- [x] locked targetの距離・速度・連続性をRecovery内で利用できることを確認する
- [x] wall、横加速度、接触、solver Recoveryの安全条件を確認する
- [x] requirements / design / tasklistを作成する

## 実装

- [x] fixed/follow速度上限の選択をcore関数へ追加する
- [x] Recoveryで既存Follow速度計算を再利用する
- [x] 既存周期debugへ速度モードを追加する
- [x] core単体テストを追加する

## 静的・単体検証

- [x] `git diff --check`
- [x] 対象coreテスト（171件成功）
- [x] `make autoware-build`（25 packages成功）
- [x] topic/service/message、launch、yamlが不変であることを確認する

## 動的検証

- [x] dev2を起動する
- [ ] d1/d2を2周以上走行する（d1 SafeStopのため1周で安全中止）
- [x] d1/d2の5トピックMCAPを確認する
- [x] Recovery回数・理由・速度モード・継続時間を抽出する
- [x] Recovery前後の速度低下、車間拡大、再試行時間を比較する
- [x] Overtake完了、SafetyStop、contact、Reverse、solver異常を確認する
- [x] lap timeとV2X rateを比較する

## 完了

- [x] `results.md`へA/B結果と暫定採用判断を記録する
- [x] ユーザー判断によりRecovery Follow引継ぎを再適用する
- [x] hairpin内側Passと復帰条件を残課題として明記する
- [x] 再適用版の対象テストとビルドを確認する
- [x] 最終差分と既存ユーザー変更を確認する
