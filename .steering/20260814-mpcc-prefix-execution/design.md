# Design

## 方針

完全 Mission を偽装せず、`progressive_entry` で表現済みの短期 prefix を active Mission の rolling replacement として扱う。

prefix の実行可否は shadow score だけに依存させず、commit 直前に純粋関数で再確認する。

- active ShiftOut/Pass
- no-return 前
- SafeSeparation 外
- candidate feasible
- body-clear checked/feasible
- target surface clearance checked and non-negative
- wall reserve が要求値以上
- body-clear の時間・距離が残 Mission budget 内
- predicted minimum speed が現在の最低要求以上

commit 後は既存 frozen plan を prefix の validated ShiftOut と短い同側 continuation に置換する。次回の optimizer miss では置換処理を行わないため、その直近 feasible plan がそのまま保持される。runtime hard fault が出た場合だけ既存 guard に判断を戻す。

## 早期切替

通常は既存の `opponent_side_replan_stable_sec` を使う。score advantage が新規閾値以上なら、hard-feasible prefix/mission に限り debounce を待たず commit する。

初期値は、今回のログで観測した advantage 1.36/1.55 を拾い、通常の 0.35 程度の差では即切替しない `1.0` とする。

## 影響範囲

- `v2x_overtake_core`: prefix commit admission、authority contract、単体テスト
- `mpc_controller_cpp`: prefix winner の execution authority、transactional replacement、decisive switch
- `config.yaml`, `config_for_cloud.yaml`: decisive score threshold

## 非対象

- full MPCC/NMPC solver への全面置換
- collision/contact guard の緩和
- Recovery/Reverse 調整
