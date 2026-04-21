# DLS ASSIST gainM Sweep — Test 1

First step of the `gainM` sweep: raise `gainM` from `0.08` (test 3
baseline) to `0.10`, keep `nullSpaceGain = 0.20` locked, all other
conservative parameters unchanged.

Prior report referenced:
- `ai/reports/DLS_ASSIST_NULLSPACE_TEST3_20260421_194652.md`
  (test 3, nsg locked at 0.20 established, residual = 2°).

## Executive Summary
- test executed: **yes**
- result: **pass**
  - final residual max at HOME: **2°** (task criterion `≤ 2°` → PASS by equality)
  - `err_target` quiescent `~0.7 – 1.0 mm` (≤ 3 mm → PASS)
  - `dx` peak per tick **9.7 mm** (≤ 15 mm → PASS; larger than test 3's 7.3 mm — expected, see below)
  - `max_step` `2°/tick` (clamp held, never exceeded → PASS)
  - sine `max_step` `1°/tick` (no oscillation amplification → PASS)
  - assistMode initial `rate`, final `rate` (restored on-disk → PASS)
- **null-space hold validated**: raising gainM by +25% did not push the
  residual back above 2°. `nullSpaceGain = 0.20` remains sufficient.
- **EE motion perceptibly larger**: target amplitudes scaled by
  `0.10/0.08 = 1.25` as theory predicts; IMU feedback on yaw shows
  `|Δq_z|` at end of yaw sweep **~80% larger** than in test 3
  (see Observations).
- no safety anomalies: no NaN, no explosion, STATUS:IDLE throughout,
  arm back at HOME `[100, 88, 93]` via post-test `SETPOSE HOME`.

## Parameters (on-Pi-only smoke-test literal, reverted after run)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.10,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.20
  }
}
```
Other on-Pi smoke-test edits (unchanged from tests 2 and 3, reverted
after run): yaw phase amplitude `+5°`, `cap()` records IMU fields.

## Environment
- repo commit (workstation): `88602f1` (test 3 report).
- Pi `ws_server` PID still **12572** (same hardened binary since the
  deploy-prep session).
- Services all active; operator dashboard on `192.168.10.80:54238`
  (single WS client throughout).
- Arm at HOME at entry and exit; IMU valid at ~90-125 Hz.

## Safety Checks
- Physical routing_config backup (by hardened smoke test):
  `routing_config.json.bak_dls_smoke_20260421_195938`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_195931`.
- Script restored from `.bak_premotion` after run; `py_compile` clean;
  parameters back to committed hardened values.
- No source-file edits committed; routing_config rolled back to
  `assistMode=rate` by the smoke test's `finally:` block.

## Motion Sequence (executed in order)
1. identity_baseline 1.5 s
2. pitch_down_5 2.0 s (head pitch +5°)
3. return_center 1.5 s
4. pitch_up_neg5 2.0 s (head pitch −5°)
5. return_center2 1.5 s
6. yaw_right_8 2.0 s (amplitude +5°, per task cap)
7. return_center3 1.5 s
8. sine_pitch_4deg 3.0 s (±4°)
9. final_identity 2.0 s
10. post-test `SETPOSE 90 90 90 90 90 90 25 RTR5`

1 656 telemetry frames captured.

## Telemetry / IMU — per-phase summary

| phase | end_t | BSG end | imu_q_{w,x,y,z} |
|---|---|---|---|
| identity_baseline | 1.51 | `[100.0, 88.0, 93.0]` | `[ 0.9725,  0.0115,  0.0055, -0.2323]` |
| pitch_down_5 | 3.52 | `[ 96.0, 88.0, 94.0]` | `[ 0.9729,  0.0154,  0.0175, -0.2301]` |
| return_center | 5.03 | `[ 99.0, 87.0, 92.0]` | `[ 0.9706,  0.0156,  0.0179, -0.2394]` |
| pitch_up_neg5 | 7.05 | `[102.0, 85.0, 88.0]` | `[ 0.9691,  0.0088, -0.0101, -0.2465]` |
| return_center2 | 8.56 | `[100.0, 85.0, 89.0]` | `[ 0.9738,  0.0082, -0.0108, -0.2272]` |
| yaw_right_8 (+5°) | 10.57 | `[105.0, 85.0, 89.0]` | `[ 0.9796,  0.0076, -0.0093, -0.2009]` |
| return_center3 | 12.07 | `[100.0, 85.0, 89.0]` | `[ 0.9735,  0.0081, -0.0104, -0.2283]` |
| sine_pitch_4deg | 15.09 | `[ 99.0, 86.0, 91.0]` | `[ 0.9708,  0.0094, -0.0065, -0.2397]` |
| final_identity | 17.32 | `[ 99.0, 86.0, 91.0]` | `[ 0.9727,  0.0103, -0.0021, -0.2321]` |

Post-test live telemetry (after `SETPOSE HOME` recovery):
`BSG=[100, 88, 93]`, `imu_q=[0.97260, 0.01276, 0.00891, -0.23212]`.

## DLS log — test gainM-1 observed ranges
- `manip`: `0.00015 – 0.00024`
- `lambda²`: `0.00400 – 0.00688` (peak well below `lambdaMax² = 0.0144`)
- `dx_cart_mm` per tick: `0.0 – 9.7` (peak the highest seen so far but
  still well under the 15 mm cap)
- `err_target_mm`: `0.4 – 8.3` (transient); quiescent `~0.5 – 1.0`
- per-joint step clamp hit 2°/tick on phase edges only
- sine-phase max step = 1°/tick (no oscillation)

Target amplitudes (vs test 3 where gainM=0.08):
- pitch_down  target Z: 312.3 mm  (test 3: 314.0) → −8.7 mm from HOME (+24% vs −7.0)
- pitch_up    target Z: 329.7 mm  (test 3: 328.0) → +8.7 mm from HOME (+24% vs +7.0)
- yaw_right   target Y: 8.7 mm    (test 3: 7.0)   → +24%

The 25% gainM increase produced ~24% increase in commanded EE
displacement — a clean linear scaling consistent with the
`target = HOME_ee + gainM * (d - forward)` formula in `head_assist_dls.py`.

## Four-test comparison

| Metric | Test 1 (nsg=0.08) | Test 2 (nsg=0.15) | Test 3 (nsg=0.20) | **Test gainM-1 (nsg=0.20, gainM=0.10)** |
|---|---|---|---|---|
| gainM | 0.08 | 0.08 | 0.08 | **0.10** |
| nullSpaceGain | 0.08 | 0.15 | 0.20 | **0.20** |
| `final_identity` BSG | `[99, 84, 88]` | `[98, 86, 90]` | `[99, 86, 91]` | **`[99, 86, 91]`** |
| per-joint residual | `[1, 4, 5]` | `[2, 2, 3]` | `[1, 2, 2]` | **`[1, 2, 2]`** |
| **residual max** | 5° | 3° | 2° | **2°** |
| task `≤ 2°` | ✗ | ✗ | ✓ | **✓** |
| `max_step` | 2° | 2° | 2° | **2°** |
| `sine max_step` | 1° | 1° | 1° | **1°** |
| `err_target` transient max | ~7.4 mm | ~6.6 mm | ~6.7 mm | **~8.3 mm** |
| `err_target` quiescent | ~1.2 mm | ~0.8 mm | ~0.5-1.0 mm | **~0.7-1.0 mm** |
| `dx` peak per tick | 8.8 mm | 7.2 mm | 7.3 mm | **9.7 mm** |
| `manip` range | 1.5-2.2 e-4 | 1.5-2.3 e-4 | 1.7-2.2 e-4 | **1.5-2.4 e-4** |
| `lambda²` range | 0.00465 – 0.00718 | 0.00419 – 0.00688 | 0.00465 – 0.00615 | **0.00400 – 0.00688** |
| direction correctness | ✓ | ✓ | ✓ | **✓** |
| safety regressions | none | none | none | **none** |
| assistMode final | rate | rate | rate | **rate** |

## Observations

- **Residual floor is robust.** Raising gainM from 0.08 to 0.10
  (+25% commanded EE reach) leaves the final residual at exactly
  `[1, 2, 2]` → 2° max, identical to test 3. `nullSpaceGain=0.20`
  absorbs the extra excursion budget without letting joints drift
  further from HOME.
- **Perceptibility clearly improved.** All commanded target
  displacements scaled linearly by ~1.25, confirmed in
  `target_mm` log entries (`314 → 312.3`, `328 → 329.7`, `7.0 → 8.7`
  on Y). During yaw the base servo reached `105°` vs `104°` in test 3
  (+1° more rotation). IMU response is ~80% stronger on the yaw
  segment: `|Δimu_q_z|` from baseline to end-of-yaw is `0.0314` in
  this test vs `0.0174` in test 3 — a better-than-linear amplification
  because the arm tip moves on a larger radius when gainM is higher.
- **`dx` peak grew proportionally.** Maximum per-tick Cartesian step
  rose from 7.3 mm → 9.7 mm (+33%), still well below the 15 mm cap
  and below even the `maxDxMmPerTick=15` clamp budget. No saturation.
- **`err_target` transient got ~24% larger** (6.7 → 8.3 mm), which is
  expected: transients happen when the target jumps between phases
  and the arm has to cover more ground before catching up. At
  `maxDqDegPerTick=2°/tick = 100°/s` the arm needs the same fixed
  time to traverse the larger amplitude, so the transient window is
  longer and accumulated error is larger — all within the same
  tracking budget.
- **Quiescent error unchanged.** Once the target stabilises, the
  arm still settles to ≤ 1 mm error as in prior tests. The null-space
  posture does not affect steady-state Cartesian tracking.
- **Damping profile unchanged.** `manip` and `lambda²` ranges are
  effectively the same as in previous tests (manip is dominated by
  the near-singular HOME geometry, not by gainM).
- **No oscillation.** Sine max step still 1°/tick; no sign that
  higher gainM combined with null-space pull is exciting any
  higher-frequency dynamics.
- **Direction correctness confirmed on all three axes** — target_mm
  signs and arm motion signs are consistent with tests 1-3.
- **`[HEAD-ASSIST] assistMode transition` log did not fire** again,
  for the same reason noted in tests 2-3 (in-process flag stayed at
  `"dls"` between runs because the dashboard does not send mode=5
  frames between tests). Implementation is by-design.
- **Firmware quiet.** STATUS:IDLE before and after, no UART errors.

## Verdict
- safe to continue tuning: **yes.**
- **gainM = 0.10 confirmed as a new operating baseline.** No safety
  tradeoff vs gainM=0.08, more perceptible EE motion, residual still
  meets the 2° task criterion.
- transient `err_target` slightly larger but unquestionably within
  budget (8.3 mm vs 15 mm cap; below the prior test 1's 7.4 mm
  headroom margin only in mm but clamp saturation is unchanged).

## Recommendation
1. **Accept `gainM = 0.10` as the new conservative baseline.**
2. **Next step: try `gainM = 0.12` or `gainM = 0.15`.** Recommended
   order:
   - **`gainM = 0.12`** first (conservative intermediate step). With
     `0.12/0.08 = 1.5` we can infer the expected `dx` peak ≈ 11 mm
     and transient `err_target` ≈ 10 mm — still within clamp budgets.
   - If clean, then **`gainM = 0.15`** (workstation factory default).
     Expected `dx` peak ≈ 13.5 mm, getting close to the 15 mm cap;
     if the cap starts triggering visibly (especially on the first
     tick of each phase) it will show up as extra lag, and we may
     need to raise `maxDxMmPerTick` to 20-25 mm to keep pace.
3. **If at `gainM = 0.12-0.15` the residual creeps above 2°**, raise
   `nullSpaceGain` to `0.22 – 0.25` as a counterweight.
4. **Do not** touch `lambdaMax`, `manipThresh`,
   `maxDqDegPerTick` yet — damping is inactive, step clamp has
   margin at current dynamics.
5. **Possible clamp relaxation** (later, not now): if larger gainM
   values start causing visible first-tick lag on phase transitions
   because `maxDxMmPerTick=15` is too tight, try raising to `20`.
   Not needed at gainM=0.10.

## Raw Evidence

Smoke-test RESULTS block:
```text
captured 1656 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [96.0, 88.0, 94.0]  step_max=2.00°
return_center end BSG:  [99.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [102.0, 85.0, 88.0]  step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 89.0]
yaw_+8 end BSG:         [105.0, 85.0, 89.0]  step_max=2.00°
return_center3 end BSG: [100.0, 85.0, 89.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 91.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=4.00°
  [FAIL] arm returns to HOME after identity — residual=2.00°   ← cosmetic (< 2.0 strict)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Final routing_config readback:
```text
assistMode = rate
assistDls  = {'gainM': 0.25, 'lambdaMax': 0.1, 'manipThresh': 0.0007,
              'maxDqDegPerTick': 3.5, 'maxDxMmPerTick': 28,
              'nullSpaceGain': 0.18}   ← operator's stored tuning, untouched
```

Post-test live telemetry (~30 s after SETPOSE recovery):
```text
BSG:   [100, 88, 93]
imu_q: [0.97260, 0.01276, 0.00891, -0.23212]
```
