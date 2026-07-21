# Design

## ログ根拠

- `output/20260721-102131/d3/autoware.log`: スタート直後、`front_distance=inf`のまま
  `side target already behind`で`Follow`へ入る。
- 同runのD1/D2: WP61でgap holdが残り0.28秒となり、WP62で旧0.5秒holdが切れて
  `ShiftOut -> Recovery`、約0.3秒後のWP63でgapが再成立する。

## 方針

1. side-only候補がrear toleranceより後方なら、保守的な`Follow`ではなく`Cruise`を返す。
2. 非有限なside進捗は従来どおり`Follow`としてfail-closedにする。
3. active gap-loss holdを0.5秒から1.0秒へ変更し、WP61-63の実測欠落をbridgeする。
4. hold対象とhard abort条件は既存実装を維持する。
