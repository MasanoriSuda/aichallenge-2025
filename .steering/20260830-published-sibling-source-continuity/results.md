# Results: published sibling source continuity

## 観測された現象

修正前run `output/20260830-160431`では、反対側current-world Bundleのcommandが
publisherを通過しMission sideが更新された直後、published Overtake alignmentだけが
古い`executed_snapshot()`を参照し`side-mismatch`になっていた。

## 問題が発生するまでの因果関係

1. same-epoch opposite siblingのcurrent-world proofが成功する。
2. exact commandがpublishされ、publication tokenがtactical sideを更新する。
3. Bundle sourceは正しく別ledgerへ記録される。
4. 次周期のretained authorityはBundle sourceを認識するが、execution alignmentは古い
   exact-executed ledgerだけを読む。
5. 新Mission sideと旧artifact sideが不一致になり、published certificate consumerが落ちる。

## 根本原因

exact-executed evidenceとpublished current-world Bundle sourceという二つの正当なledgerに対し、
最新publication sourceを選ぶ規約がconsumerごとに異なっていたことが原因である。

## 実施した変更

- Storeへ`latest_published_source_snapshot()`を追加した。
- 一つのmutex下で、Bundle sourceがあればそれを、なければexact-executed planを返す。
- source kind、plan/sibling、publication decision、control-origin clock、artifact cursorを一組で返す。
- published Overtake alignmentをこのatomic snapshotへ接続した。
- source kindとsource sideをdecision logへ追加した。
- ログ発火はactive/intent/side/reasonの状態変化に限定し、exact/Bundleの通常交互利用では出さない。

stateless Bundleのsource planを`mark_executed()`へ昇格する変更は行っていない。

## 静的検証

- `git diff --check`: 成功
- isolated source contract: 87 passed
- `make autoware-build`: 25 packages成功
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 2243 tests、0 errors、0 failures、0 skipped
- `joycon_contract_guard/package.xml`欠損は既知の無関係なtest-result診断

## 動的検証

run: `output/20260830-162637/d1/autoware.log`

### sibling adoption 1

- decision 3797: `side=1->-1`, source sequence 2786, phase Pass
- 同decisionの次のalignment:
  `active=1 intent=pass source=current-world-bundle source_side=-1`
- published alignment `side-mismatch`: 0

### sibling adoption 2

- decision 5269: `side=-1->1`, source sequence 4206, phase Pass
- 同decisionの次のalignment:
  `active=1 intent=pass source=current-world-bundle source_side=1`
- 次周期も`published-bundle-reproved` authorityを継続
- published alignment `side-mismatch`: 0

### 集計

| 観測 | 件数 |
|---|---:|
| opposite sibling published/adopted | 2 |
| adoption直後のpublished alignment成功 | 2 |
| published alignment side-mismatch | 0 |
| Bundle source record rejection | 0 |
| Idle -> ShiftOut | 3 |
| ShiftOut -> Pass | 3 |
| Pass -> Return | 0 |
| Pass -> Recovery | 3 |

ログ中の`lease=side-mismatch` 1件はMPCC-lite async cacheの旧side診断であり、published
alignmentの拒否ではない。

## 残っている懸念

今回のrunでは3 episodeすべてPassまで進んだが、Returnへ到達しなかった。

- episode 1: `committed pass longitudinal progress stalled`
- episode 2: `actual footprint wall margin violated`
- episode 3: `committed pass longitudinal progress stalled`

これはopposite sibling publication continuityとは独立したPass完遂故障である。今回のSliceへ
progress timeout、wall margin、fallback、solver設定変更を混ぜていない。

また、Pass中に`published trajectory resampling failed`となる区間があり、authority自体は別の
current-world proofで継続している。このfailure familyも、source chronologyではなくremaining
geometry/cursorの問題として別監査が必要である。

## 次回試走で確認すべき項目

- sibling adoption後のPass progress、front/rear relative progress、speed ownerを同一decisionで追う。
- progress stallがcandidate terminal不足、前進速度authority、相手予測のいずれから始まるか分類する。
- wall-margin episodeはpublished trajectoryと実測footprintの最初の乖離位置を比較する。
- 6周acceptanceでPass -> Return -> Idle完遂率とRecovery原因を集計する。
