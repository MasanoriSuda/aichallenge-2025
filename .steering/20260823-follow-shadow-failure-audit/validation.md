# Validation

## Baseline dynamic evidence

- Run: `output/20260823-135631`
- High-speed moving target: ego 8.23 m/s、`v_upper0=5.43 m/s`でmaximum iterations、
  primal residual 1.97。
- Stopped target: ego 8.87--8.98 m/s、`v_upper0=3.0 m/s`でmaximum iterations、
  primal residual 4.57--4.79。
- Solved-but-rejected sample: acceleration stage 0が1.37431 m/s^2、violation
  0.00430688、row tolerance 0.00237431。

## Static validation

- `make autoware-build`: success。
- `test_race_mpcc_foundation`: 20/20 passed。
- 追加回帰: 9.0 m/sから0.1秒刻み、最大制動3.0 m/s^2のhard upperが
  8.7、8.4、8.1 m/sとなり、soft reference 5.0 m/sを維持する。

## Corrected dynamic evidence

- Run: `output/20260823-140735`
- ego 8.215 m/s、target 4.497 m/sでは`v_ref0=5.297 m/s`に対し
  `v_upper0=7.946 m/s`となり、41/41 cycles acceptedのwindowを確認。
- baselineのような数秒連続0% acceptedは解消した。
- target gap 5--7 mの最大制動境界ではmaximum iterationsとrow-wise primal rejectが
  残り、最悪windowは11/38 accepted。その後は82.9%、97.6%、100%へ回復した。

## Gate result

- Reachability root cause correction: pass。
- Follow shadow observability: pass。
- Follow production authority promotion: fail/保留。
- 次のblocking cause: OSQP global terminationとrow-wise execution certificateの不一致。
