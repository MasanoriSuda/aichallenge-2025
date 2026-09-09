# Tasks

本sliceは完遂計画の具体化とモデル比較の証拠を残す。MPCC全体は未完。

- [x] 開始HEAD ae2efe6a、production1c4f377e、既存変更を確認。
- [x] wheel/brake/rest、wire入力と速度微分のproducerを読む。
- [x] 現行native、flat offset、停止入力依存の反例を保存。
- [x] 単車6周の診断runとraw source時刻付き入力を保存。
- [x] 学習窓と保留窓、独立dev2を分離してモデル比較。
- [x] raw Euler差分の巻き戻りを特定し、解析補正前後を保存。
- [x] 最小dev2観測で実効mass/skid/grip/COM/inertia/車輪接地を確認。
- [x] 原本DLL、保護JSONを確認し、全runtimeを停止。
- [x] 本番採用できる案と未確定事項を区別し、対象コード・検証・完遂順序を文書化。
- [x] evidence封印、registry、正本文書・リンク・構文確認。
- [x] 必要分をlocal commitへまとめる（本sliceのcommitに記録）。

以下は[完遂計画](completion-plan.md)へ引き継ぐ。今回の観測合格でチェックしない。

- [ ] 因果的なモデル比較と適用時刻の不確かさの契約を確定。
- [ ] 採用モデルの失敗test→共通producer・schema・proof・asyncの修正。
- [ ] native/package/build/replay、単車・限定dev2を合格。
- [ ] 移動中の複数tick Stop→停止→再発進、左右Pass/Return、残り全intent/async。
- [ ] 固定campaign、dev3/dev4、gate1/2/3、同一提出物からeval、最終文書・commit。
