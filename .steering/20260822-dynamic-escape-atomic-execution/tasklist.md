# Tasklist

- [x] 最新走行ログから DynamicEscape path/control authority のチャタリングを確認する
- [x] retained execution のstage cursor計算を純粋関数化して単体試験を追加する
- [x] live candidateの即時物理昇格とretained horizon保持を実装する
- [x] handoff決定ログへ実行解の由来と残りhorizonを追加する
- [x] 対象単体試験を実行する（55 tests passed）
- [x] ROS 2 packageをビルドする（25 packages finished）
- [x] package全体の回帰試験を実行する（32/32 test targets passed）
- [x] `colcon test-result --verbose` を確認する（1517 tests, 0 errors, 0 failures）
- [x] 差分をレビューしてコミットする

`pre-commit` はホストにコマンドが存在しないため未実行。Docker内のpackage testに含まれる既存lint/test targetは全件通過した。

## 次回試走で確認するログ

- `incoming_admitted=1/promoted=1` の周期で `hold=0` になっていること
- 再評価中に `published_source=retained-stage` が出ること
- `stage` が経過時間とともに増え、`remaining_control` が減ること
- `published_source=last-steering-hold` が長時間継続しないこと
- RVizのDynamicEscape予測線が1周期ごとに生成・消失しないこと
