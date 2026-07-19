# Results

## Static verification

- `make autoware-build`: 成功。25 packages。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功。17 test targets。
- `colcon test-result --verbose`: 431 tests、0 errors、0 failures、0 skipped。
  - 既存の`build/joycon_contract_guard/package.xml`欠落についてtest-result parser警告が出るが、
    対象packageのtest結果には失敗なし。

## Covered behavior

- 積極復旧中はLowSpeedRejoinがsolver healthへ依存しない。
- attempt limit等の回復可能SafeStopは待機後にbudgetを初期化してStopAndConfirmへ戻る。
- invalid inputは積極復旧でもSafeStopへラッチする。
- 積極復旧を`simulation_only: false`と組み合わせたCoreConfigは起動時に拒否する。

## Runtime experiment

### Run 1: `output/20260718-225500`

約6分のdev3を実施した。

- 3台ともrace開始後に発進した。
- D2のReverseは約0.7秒で0.164 m移動し、REVERSE gearで負の加速度を駆動側にする設定を実走確認した。
- D3は`SAFE_STOP(clearance_wait_timed_out)`から0.5秒後に`aggressive_retry`へ遷移した。
- D1 / D2 / D3はそれぞれ34 / 27 / 31回SafeStopから再試行し、永久ラッチは解消した。
- D1 / D2 / D3の最大episode移動距離は0.778 / 0.841 / 0.308 mだった。
- `rejoin_complete`は3台とも0回だった。
- 最大横偏差はD1 5.153 m、D2 3.727 m、D3 5.132 m。短いReverse/Forwardを反復して
  通常周回へ戻れず、終了時は全車0 m/sだった。

初回結果は「永久SafeStop解除」はPass、「競技走行復帰」はFail。現在footprintがclearでも、
0.8 m先のrejoin sweepがwallと交差して`LOW_SPEED_REJOIN`へ入れないことを確認した。

### Run 2: `output/20260718-230630`

3回以上の失敗後、現在footprintがclearなら予測sweepを緩和する
`aggressive_force_after_retries: 3`を追加し、再ビルド・全test後に再実験した。

- 強制rejoinはD3で発火した。横偏差は2.824 mから終了時0.582 mまで縮小した。
- D2の通常rejoinは約1.0 m/sで横偏差を-0.628 mから+0.517 mまで動かしたが、
  heading errorが2.169 rad残ったまま反対側のwallへ達した。
- D1 / D2 / D3のSafeStop再試行は2 / 2 / 7回、LowSpeedRejoinは0 / 2 / 23回だった。
- `rejoin_complete`は再び3台とも0回だった。
- 終了時横偏差はD1 2.972 m、D2 2.348 m、D3 0.582 m、終了時速度はほぼ0 m/sだった。

二段目は経路方向へ車を動かす効果を確認したが、姿勢誤差が大きい車両の向きを直す経路を作れず、
競技走行復帰はFailのままである。以後は無制限なretry追加ではなく、接触前の衝突回避強化と、
位置だけでなく姿勢を先に合わせる複数点rejoin plannerが必要である。

## Verification checklist

- REVERSE中のacceleration command、signed velocity、4秒あたりの移動距離。
- `solver_unsafe`後もLowSpeedRejoinを継続すること。
- `aggressive_retry`後に候補とepisode距離がresetされること。
- retry奇数回で、clearなforward fallbackがあれば選択されること。
- レース終了時の周回、順位、最終速度、永久SafeStop有無。
