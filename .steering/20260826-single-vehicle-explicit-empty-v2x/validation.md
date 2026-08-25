# Validation

## Static and package tests

```text
python3 -m pytest -q \
  .steering/20260826-single-vehicle-explicit-empty-v2x/test_host_startup_contract.py \
  aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/test/test_single_vehicle_v2x_contract.py
4 passed

colcon test --packages-select aichallenge_system_launch
3 passed

make autoware-build
25 packages finished; build successful

xmllint --noout <changed launch XML files>
passed

git diff --check
passed

python3 -m flake8 <new publisher and contract tests>
passed
```

## Dynamic acceptance

### make dev

- run: `output/20260826-050706`
- d1 state: `start`
- max observed speed: `8.68 m/s`
- V2X message vehicles: `0`
- `single_vehicle_empty_v2x_publisher`: started
- producer start後の`dynamic-observation-unavailable`: `0`
- verdict: pass

### make dev2

- run: `output/20260826-050836`
- d1/d2 state: `start` / `start`
- max observed speed: `5.31 / 4.51 m/s`
- V2X message vehicles: `1 / 1`
- `single_vehicle_empty_v2x_publisher`: both absent
- d2のnative V2X初回受信前に`dynamic-observation-unavailable`が1回あるが、継続せず
  Healthyへ遷移して発進した。
- verdict: pass

### make dev3

- run: `output/20260826-050947`
- d1/d2/d3 state: all `start`
- max observed speed: `7.14 / 4.61 / 6.58 m/s`
- V2X message vehicles: `2 / 2 / 2`
- `single_vehicle_empty_v2x_publisher`: all absent
- `dynamic-observation-unavailable`: `0 / 0 / 0`
- verdict: pass

## Interface compatibility

- Domain 0 management ownership: unchanged
- vehicle Domain 1..N: unchanged
- `/v2x/vehicle_positions` name/type: unchanged
- domain bridge: not added
- `/admin/awsim/start` owner: unchanged
- participant command/topic contract: unchanged
- result JSON/submission tar contract: unchanged

## Remaining concerns

- 本producerはローカル/評価system harnessの単車simulation契約であり、参加者tar.gzには含まれない。
- 公式2026環境で単車シナリオを提供する場合のempty V2X契約は公式仕様確認が必要。
- 本Sliceは発進契約のみを修復する。既存のsteering/cursor/solver rejectは別のMPCC品質課題である。
