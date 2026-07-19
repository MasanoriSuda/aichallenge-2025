# Results

## 実装結果

- 前方検出24 mとgeneric Follow速度cap 5 mを分離した。
- 5 mより遠方ではgeneric capを掛けず、5 m以内の移動前車では
  `front_speed + moving_margin`、低速前車では距離停止上限と固定capの小さい方を使う。
- front risk、curve risk、front decel guard、SafetyBrakeは距離gate外でも維持した。
- active Passのgap一時欠落ではgeneric Follow capを抑止した。
- debugへ`follow_cap`、`follow_moving`、`follow_cap_dist`を追加した。

## 検証

- `config.yaml`をPyYAMLで読み込み: 成功
- `git diff --check`: 成功
- `make autoware-build`: 25 packages成功
- `test_v2x_overtake_core`: 87 tests成功（新規5 testsを含む）

全履歴を対象にした`colcon test-result --verbose`は、今回の対象テスト成功後に、既存の
`test_path_core`失敗結果1件と欠損した`joycon_contract_guard/package.xml`を拾って終了コード1に
なった。対象テスト単独を再実行し、終了コード0を確認した。

dev3走行A/Bはこの修正ターンでは実施していない。次回走行ではV2X debugの`fd`と
`follow_cap`を見て、5 m超で0、5 m以内で1へ切り替わることを確認する。
