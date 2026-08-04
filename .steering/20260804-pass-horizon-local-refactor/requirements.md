# Pass horizon 局所リファクタリング

## 目的

Pass 継続、full-path preflight、SafeSeparation 移行が
`MPC::update_overtake_line()` 内で密結合になっている箇所を、次の
Pass/Return 分離修正に先立って読み分けられる状態にする。

## 制約

- 走行挙動、判定条件、パラメータ、ログ文言を変更しない。
- ROS 2 topic/service/message および提出インターフェースを変更しない。
- 変更範囲は `multi_purpose_mpc_ros` の追い越し Pass horizon 処理に限定する。
- ユーザーの既存変更と走行成果物には触れない。

## 完了条件

- committed Pass の full-path preflight 呼び出しが名前付きメソッドへ分離されている。
- Pass horizon action の入力生成が名前付きメソッドへ分離されている。
- SafeSeparation の開始処理が名前付きメソッドへ分離されている。
- 短期安全判定を同一制御周期で重複計算しない。
- 対象パッケージのビルドと既存テストが成功する。
