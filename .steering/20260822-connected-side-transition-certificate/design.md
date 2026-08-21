# Design

## 問題

従来は、現在の横位置を含み要求側へ少しでも広がる free interval を見つけると、そのステージを `gateway-entered` として横断完了扱いにしていた。次ステージから要求側だけを強制するため、実際には横断途中でも `forced-side-empty-after-gateway` になり得た。

## 方針

反対側遷移を次の状態に分離する。

1. `Searching`: 現在位置と接続する gateway を探索する。
2. `Crossing`: gateway 内で横目標を要求側へ進めながら、各ステージの接続性を検証する。
3. `Certified`: 計画横位置が要求側へ到達し、そこから設定距離ぶん接続回廊が継続した後に、要求側 homotopy を強制する。

`Crossing` 中に接続 gateway が消えた候補は、その場で planner infeasible とする。設定済みの transition distance は探索期限ではなく、要求側到達後の connected-prefix 証明距離として利用する。

## ログ

候補ログへ以下を追加する。

- crossing started
- requested side reached
- transition certified
- certified-until index/distance
- first disconnect index/distance

tracking ログには committed branch (`primary` / `alternate`) を追加する。これにより alternate を一度評価した事実と、実際に tracking へ渡った branch を区別する。

## 対象外

- exact shadow tracking QP による採用前検証
- warm start の topology fingerprint 対応
- Recovery から通常制御への heading/rejoin gate

これらは今回のログで残事象を確認した後の別変更とする。
