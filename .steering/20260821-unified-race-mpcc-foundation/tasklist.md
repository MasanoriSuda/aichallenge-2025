# Unified Race MPCC foundation tasklist

- [x] 現行HEADと最新ログの構造的不具合を総点検する
- [x] requirements / designを作成する
- [x] pure C++ `StageGeometry` helperと非等間隔テストを追加する
- [x] MPC/MPCC/wall/certificateを共通stage geometryへ接続する
- [x] side別persistent branch solver contextを追加する
- [x] target provenanceとasync adoption validationを追加する
- [x] Race MPCC homotopy shadow schemaと集約ログを追加する
- [x] buildと関連単体テストを実行する
- [x] docs/spec/mpc-integration.mdを更新する
- [x] 差分をレビューしコミットする

## Dynamic verification left to user

- [ ] `make dev2`で通常走行が現行と同等に継続する
- [ ] shadowログでstage geometryが連続する
- [ ] Left / Rightの連続評価でwarm-startが再利用される
- [ ] stale target resultが理由付きで棄却される

## Static verification

- `make autoware-build`: 25 packages build successful
- `ctest --output-on-failure`: 32 / 32 passed
- ROS topic / service / launch / result schema: no changes
- Current runtime authority: unchanged (`Race MPCC shadow` only)
