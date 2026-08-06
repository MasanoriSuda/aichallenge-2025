# Design

## 1. 並走完遂の明示判定

controller 内に散らばっていた「同じ側のまま前へ出てよい」条件を pure policy へまとめる。
許可条件は以下とする。

- competition attack mode が有効
- Pass、minimum-motion corridor、front-cap release が有効
- target ID と course progress が連続し、position jump がない
- target が設定された forward window（現行 3 m）以内
- 現在車体が非重複、または既存 0.30 秒確認中の単発 overlap
- 壁接触・壁観測欠損・pass-side intrusion・EmergencyBrake・solver recovery がない

この判定は初回 front-cap 解放には使わない。既に検証済みの Pass の完遂だけを対象とする。

## 2. 横経路の固定

並走完遂が有効になった周期から、predicted-overlap による same-side lateral replacement と
continuous outer transition を抑止する。相手の横予測に追従してさらに壁側へ押し出すのではなく、
既に解放済みの横 corridor を保持して前後関係だけを反転させる。

## 3. SafeSeparation の早期利用

Pass horizon が尽きてからではなく、並走完遂条件が成立した時点で SafeSeparation を開始する。
これにより target speed + 2.0 m/s を上限とする既存 forward escape を直ちに利用する。

rear-clear が成立すれば Return、確認済み overlap や物理 guard 不成立なら Recovery とする。

## 4. 絶対上限と局所完遂枠

SafeSeparation 開始後に forward escape が現在も許可されている場合だけ、Pass 全体の絶対距離・
時間上限より現在の SafeSeparation 局所枠を優先する。これにより 32 m 直前で開始した完遂処理も、
現行 5 秒 / 12 m の局所枠までは継続できる。

絶対上限到達後は progress extension を許可しないため、局所枠を繰り返し再設定することはない。
forward escape が失効した周期は絶対上限を即時適用する。

## 5. Return 条件

Pass horizon の hard-limit 分岐も、target center が 0 m を下回っただけでは Return しない。
既存の `rear_clear_confirmed` と return corridor 成立を要求し、未達なら同側の前方完遂を継続する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: 並走完遂 pure policy と SafeSeparation 上限優先順位
- `mpc_controller_cpp.cpp`: 完遂判定、横再計画抑止、SafeSeparation 早期開始
- `test_v2x_overtake_core.cpp`: 前方完遂、overlap grace、絶対上限の回帰試験
- `docs/spec/mpc-integration.md`: committed Pass の完遂優先順位

設定値とROS interfaceは変更しない。
