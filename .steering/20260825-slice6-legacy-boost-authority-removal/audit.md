# Audit

## Evidence before change

- 提出launchは`use_boost_acceleration=false`だが、C++ nodeはparameterを宣言し、true時に
  `/boost_commander/command`へ別messageをpublishできる。
- C++ control callbackにはcanonical normal branchとは別に、速度と操舵閾値から
  `acc=500.0`等を選ぶlegacy normal branchが残る。
- `boost_commander`はlegacy messageを受けて最終`/control/command/control_cmd`をpublishする。
- 同じflagはStuck Recovery reverse actuationと2026公式Boostを無効にする。
- 比較用Python controllerとlaunchにも同じ経路が残る。
- `SolverDerivedBypass`はproduction producerがなく、非canonical拒否試験用の負例であるため本Sliceの
  削除対象ではない。

## Root cause

2025の加速実験用中継nodeが「無効なoption」として残り、canonical MPCC導入後もfinal publisherと
acceleration authorityを再所有できる構造が維持されていること。
