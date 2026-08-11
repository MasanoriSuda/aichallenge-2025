# Design

## 原因

`update_overtake_line()` はSafeSeparationの進捗喪失時に、targetが前方へ十分離れ、
車体・予測sweep・Return corridorがclearなら速度を維持したReturnを選ぶ。その後、同じ
`behavior_output` を使って再帰的にReturn経路を生成する。

現行のReturn早期再捕捉はReturnの開始理由を区別しない。このため、tactical
revalidationでReturnを選んだ直後にも `gap_available` がtrueのまま残り、同じ周期にPassへ
戻る。最新ログの9件はすべてこの並びで発生している。

## 方針

### Return ownership

Returnへ次の方針を付与する。

- `SameTargetEarly`: 従来どおり、Return初期に同一target・同一sideの実行機会が戻ればPassへ戻れる。
- `FinishReturn`: Return完了まで同一targetの再捕捉を禁止する。

通常のReturn遷移は互換性維持のため `SameTargetEarly` を既定とする。SafeSeparationの
`target clear ahead` によるtactical disengagementだけ `FinishReturn` を指定する。

### Pure policy

`ReacquireRequest` にReturn ownershipを表すboolを追加し、
`can_reacquire_during_return()` の必須条件にする。controller内だけで条件分岐せず、境界を
単体テストで固定する。

## 非対象

- 追い越しentry admissionの変更
- full Mission preflightの段階化
- Setup/Pressure候補の追加
- Backoff/Cutback戦術の追加
- 速度・クリアランスの攻撃化

これらは本修正の動的確認後、別ステアリングで扱う。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: Return再捕捉policy入力
- `mpc_controller_cpp.cpp`: Return ownership状態とtactical Returnへの接続
- `test/test_v2x_overtake_core.cpp`: policy境界テスト
- 外部interface、yaml、launch: 変更なし
