# Requirements

## 目的

動的障害物回避で反対側 branch を採用するとき、反対側へ通じる区間へ入っただけで横断完了と判定しない。実行可能な接続回廊が複数ステージ継続し、計画横位置が要求側へ到達したことを確認してから branch を採用可能にする。

## 制約

- 追い越し・壁余裕・OSQP の設定値は変更しない。
- ROS 2 topic、message、service、launch、提出物の契約は変更しない。
- 通常走行と既存の Recovery 経路は変更しない。
- `aichallenge/result-summary.json` の既存変更は対象外とする。

## 完了条件

- gateway 入口と横断証明が別状態になる。
- 横断中に接続回廊を失った候補は採用前に拒否される。
- 要求側到達後、設定距離ぶん接続性を確認した候補だけを横断済みとする。
- 候補ログに gateway、証明区間、最初の切断位置が出る。
- tracking 失敗ログから primary/alternate のどちらを採用したか追跡できる。
- 関連単体テストと package build が成功する。
