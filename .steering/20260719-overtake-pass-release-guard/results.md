# Results

## 実装結果

- ShiftOut -> Passを`shift_distance`と横目標収束のAND条件へ変更した。
- locked targetが前方にいるearly Passでは、前方車速度基準のcapを維持する。
- active OvertakeLineが保持される間は同じcapをMPC速度制約にも適用する。
- 安全条件内のlocked targetは、一時的なgap再評価不成立だけではRecoveryへ戻さない。
- ShiftOut距離適応を有効化し、closing speedを0.0〜1.5 m/sへ変更した。
- ShiftOut / Passの横参照を明示OvertakeLineだけに統一し、phase累積距離でrampを進めるようにした。
- Pass目標をlocked targetの共通コース横位置基準とし、前後重なり解除も共通コース横差でlatchする。
- latch前Passのclosing speedを0.5 m/sへ制限した。
- 後方side targetへの新規ShiftOutを拒否し、latch済みPassはhard境界で継続できるようにした。

## 検証

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 82 tests、failure 0、error 0
- package test集計は既存`test_path_core` 1件と欠損した別build XMLを拾い終了コード1。
  対象gtest XMLは`tests=82 failures=0 errors=0`を確認した。

## dev3最終実験

- 最終run: `output/20260719-223908`
- D2はShiftOutからPassへ移行し、WP154付近で`hard_continue=1`かつ`closing=0.50`を確認した。
- WP156で`Pass front-overlap exclusion latched, target=d3, lateral=-1.02`を確認した。
- D3では後方side targetを`side target already behind`として新規追い越しから除外した。
- 実験後に`make down`を実行し、dev3とsimulatorコンテナを停止した。

## 残課題

- 最終runではPassと前後重なり解除までは確認できたが、安定したReturn完了と順位改善は未確認。
- D2の開始直後WP29ではOSQP maximum iterationsが残る。ユーザー方針により開始直後の混雑は今回の追い越し実験から除外した。
- 追加修正・追加runは行わず、この実験系列はここで一旦終了する。
