# E2E Dataset Contract Task List

- [x] 現extractor / dataset / train flowを監査
- [x] 一意sequence identityと衝突検査
- [x] 同期閾値と常時delta保存
- [x] run-level deterministic split
- [x] metadata / failure summary
- [x] extractor / trainer unit tests
- [x] baseline bag 2本で実抽出
- [x] train/val overlap監査
- [x] label provenanceをtrainerまで強制
- [x] READMEと正本仕様更新
- [x] Docker内test / py_compile
- [x] 変更をコミット

## Verification

- unit tests: 14 passed
- 実bag:
  - `output/20260901-021032`: 8,483 accepted / 0 sync reject
  - `output/20260901-022146`: 1,949 accepted / 1 sync reject
- 同名`rosbag2_autoware` 2本を別sequence IDへ抽出
- `label_source=student`を既定trainerが拒否
- 実教師bagはまだ収集していないため、このSliceでは再学習しない
