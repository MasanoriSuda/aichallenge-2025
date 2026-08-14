# Tasklist

- [x] 最新実走のDP coverageと失敗遷移を確認する
- [x] 短いDP列の生成原因を特定する
- [x] 要件・設計を作成する
- [x] DP専用full-horizon corridor profileを実装する
- [x] same-target/same-side rolling refreshを実装する
- [x] runtime logを追加する
- [x] 単体テストを追加する
- [x] `make autoware-build`相当のDockerビルドを実行する
- [x] package testを実行する

## 動的確認（ユーザー実施）

- [ ] freeze時のDPが20点以上、rear-clear想定距離まで生成される
- [ ] `DP execution rolling refresh`がShiftOut/Pass中に観測できる
- [ ] `optimized horizon escaped target separation bounds`が減る
- [ ] `Pass -> Return -> Idle`完遂率が上がる
- [ ] 壁接触、wall Recovery、solver failureが増えていない
