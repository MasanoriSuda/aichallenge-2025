# Task list

- [x] 最新ログと現行entry admissionを照合する
- [x] Mission総時間clockの初期化を実装する
- [x] 停止車用entry-speed overrideを実装する
- [x] 設定と診断ログを追加する
- [x] core単体テストを追加する
- [x] package build/testを実行する
- [x] 差分と実走確認点を整理する

## Definition of Done

- start-grid breakoutでMission elapsedがNaNにならない
- 停止確認済みの車両に対し、完全Missionが成立すれば速度確認待ちなしで実行へ入れる
- Mission不成立、3 m未満、EmergencyBrake、停止未確認ではoverrideされない
- 既存core testsを含む対象packageのtestが成功する

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  922 tests、0 errors、0 failures、0 skipped
- 実走では次を確認する:
  - `elapsed=nan/15.00 s`が0件
  - `V2X confirmed stationary blocker direct Overtake handoff`の直後にOvertakeへ遷移
  - Mission不成立または前方3 m未満では従来どおりSafetyBrake/Followを維持
