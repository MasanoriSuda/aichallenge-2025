# E2E Teacher Collection Task List

- [x] controller選択のlaunch/data flow監査
- [x] system launchへ明示controller引数を追加
- [x] Docker / run scriptへ許可値付き伝播を追加
- [x] deterministic teacher scenarioを追加
- [x] launch contract testを追加
- [x] build / unit test
- [x] `make e2e-teacher`で3周動的確認
- [x] teacher bag topic/count確認
- [x] `label_source=mpc`で実抽出
- [x] Sliceをコミット

## Verification

- build: 25 packages passed
- related package tests: 17 tests passed
- workspace test summary: 2,286 tests / 0 errors / 0 failures
- teacher run: `output/20260901-024545`
  - system / submit launchともに`control_method: mpc`
  - AWSIM state: `Finish`（3 laps scenario）
  - controller lap log: 58.713 s / 57.688 s（finish crossingはLap 3として記録されない）
  - bag duration: 172.141 s
  - LiDAR: 3,439 messages
  - control command: 6,887 messages
- dataset extraction:
  - 3,439 accepted / 0 rejected
  - sync delta mean 8.08 ms / p95 12.17 ms / max 16.50 ms
  - `label_source=mpc`をtrainer contractが受理
- E2E scenarioにはresult JSON producerがなく、AWSIM `Finish`とbag finalizeを完了根拠とした
