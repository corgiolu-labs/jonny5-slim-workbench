# DLS ASSIST Null-Space Tuning Test 3

Third controlled motion test on `jonny5@192.168.10.92`, varying only
`nullSpaceGain` from `0.15` (test 2) to `0.20`. Everything else
identical to tests 1 and 2.

Prior reports referenced:
- `ai/reports/DLS_ASSIST_MOTION_TEST_20260421_191712.md` (test 1, nsg=0.08)
- `ai/reports/DLS_ASSIST_NULLSPACE_TEST_20260421_192507.md` (test 2, nsg=0.15)

## Executive Summary
- test executed: **yes**
- result: **pass at task threshold**
  - final residual max at HOME = **2°** (task criterion: `≤ 2°` → PASS by equality)
  - the in-script smoke-test verdict still reports `FAIL residual=2.00°`
    because its internal comparator uses strict `< 2.0`; the task's
    own criterion is `≤ 2°`, so this is a PASS per the task and a
    "on the line" per the smoke-test cosmetic verdict.
- `err_target` quiescent **~0.5 - 1.0 mm** (≤ 3 mm target → PASS)
- `max_step` = **2°/tick** (clamp held, never exceeded → PASS)
- `sine max_step` = **1°/tick** (no oscillation amplification → PASS)
- direction correctness: confirmed on all three axes
- assistMode initial: `rate`; assistMode final: `rate` (restored by
  smoke-test `finally:` block; verified on disk after the test)
- no safety anomalies: no NaN, no explosion, STATUS:IDLE throughout,
  arm back at HOME `[100, 88, 93]` via post-test `SETPOSE HOME`.

## Parameters (on-Pi-only smoke-test literal, reverted after run)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.08,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.20
  }
}
```
Other on-Pi smoke-test edits (same as test 2, reverted after run):
yaw phase amplitude `+5°`, `cap()` records IMU fields.

## Environment
- repo commit (workstation): `94922ac` (test 2 report).
- Pi `ws_server` PID **12572** (same process as tests 1 and 2 —
  hardened binary, env `J5VR_TELEMETRY_FILE=/dev/shm/j5vr_telemetry.json`).
- Services all active (`jonny5-ws-teleop`, `jonny5-spi-j5vr`, HTTPS,
  MediaMTX).
- Single WS client on 8557: `192.168.10.80:54238` (operator dashboard
  `MSI.lan`, unchanged across all three tests).
- Arm at HOME at entry and exit.
- IMU `imu_valid=True` throughout; rate ~90-125 Hz.

## Safety Checks
- Smoke-test physical routing_config backup (by the hardened script):
  `routing_config.json.bak_dls_smoke_20260421_194537`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_194530`.
- Smoke-test script restored from the `.bak_premotion` copy after the
  run; `py_compile` clean; parameter literals back to committed
  hardened values.
- No routing_config contents committed; no runtime config changed
  persistently (file restored to `assistMode=rate`).

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

1 605 telemetry frames captured.

## Telemetry / IMU — test 3 per-phase summary

| phase | end_t | BSG end | imu_q_{w,x,y,z} |
|---|---|---|---|
| identity_baseline | 1.49 | `[100.0, 88.0, 93.0]` | `[ 0.9729, 0.0101,  0.0011, -0.2311]` |
| pitch_down_5 | 3.49 | `[ 96.0, 88.0, 93.0]` | `[ 0.9724, 0.0097,  0.0008, -0.2334]` |
| return_center | 5.01 | `[ 99.0, 87.0, 92.0]` | `[ 0.9707, 0.0103,  0.0007, -0.2401]` |
| pitch_up_neg5 | 7.02 | `[102.0, 85.0, 89.0]` | `[ 0.9681, 0.0082, -0.0165, -0.2497]` |
| return_center2 | 8.54 | `[100.0, 85.0, 89.0]` | `[ 0.9724, 0.0067, -0.0165, -0.2330]` |
| yaw_right_8 (+5°) | 10.54 | `[104.0, 85.0, 89.0]` | `[ 0.9767, 0.0063, -0.0153, -0.2137]` |
| return_center3 | 12.07 | `[100.0, 85.0, 89.0]` | `[ 0.9724, 0.0067, -0.0156, -0.2327]` |
| sine_pitch_4deg | 15.08 | `[ 99.0, 86.0, 91.0]` | `[ 0.9700, 0.0071, -0.0122, -0.2426]` |
| final_identity | 17.29 | `[ 99.0, 86.0, 91.0]` | `[ 0.9721, 0.0068, -0.0121, -0.2341]` |

Post-test telemetry (verified after `SETPOSE HOME` recovery):
`BSG=[100, 88, 93]`, `imu_q=[0.97229, 0.01068, 0.00317, -0.23352]`.

## DLS log — test 3 observed ranges
From `[HEAD-ASSIST-DLS]` journal entries during the test:
- `manip`: `0.00017 – 0.00022`
- `lambda²`: `0.00465 – 0.00615` (peak well under `lambdaMax² = 0.0144`)
- `dx_cart_mm` per tick: `0.0 – 7.3` (well under 15 mm cap)
- `err_target_mm`: `0.4 – 6.7` (transient); quiescent `~0.5 – 1.0`
- per-joint step clamp hit 2°/tick on phase edges, never exceeded
- sine-phase max step = 1°/tick (no oscillation)

## Three-test side-by-side comparison

| Metric | Test 1 (nsg=0.08) | Test 2 (nsg=0.15) | Test 3 (nsg=0.20) |
|---|---|---|---|
| `final_identity` BSG | `[99, 84, 88]` | `[98, 86, 90]` | **`[99, 86, 91]`** |
| per-joint residual `|ΔBSG|` | `[1, 4, 5]` | `[2, 2, 3]` | **`[1, 2, 2]`** |
| **residual max (deg)** | **5** | **3** | **2** |
| task `≤ 2°` success criterion | ✗ | ✗ | **✓** (at threshold) |
| smoke-test internal verdict (`< 2.0`) | FAIL 5° | FAIL 3° | FAIL 2° (on-line) |
| `max_step` (clamp reached) | 2° | 2° | 2° |
| `sine_pitch_4deg` max_step | 1° | 1° | 1° |
| `err_target` transient max | ~7.4 mm | ~6.6 mm | ~6.7 mm |
| `err_target` quiescent | ~1.2 mm | ~0.8 mm | **~0.5 – 1.0 mm** |
| `manip` range | 1.5e-4 – 2.2e-4 | 1.5e-4 – 2.3e-4 | 1.7e-4 – 2.2e-4 |
| `lambda²` range | 0.00465 – 0.00718 | 0.00419 – 0.00688 | 0.00465 – 0.00615 |
| `dx` peak per tick | 8.8 mm | 7.2 mm | 7.3 mm |
| direction correctness | ✓ | ✓ | ✓ |
| safety regressions | none | none | none |
| assistMode final | rate | rate | rate |

Residual shrank monotonically: `5° → 3° → 2°` for `nsg = 0.08 → 0.15
→ 0.20` — a slope of roughly `−1.0° per +0.05 nsg`. None of the
auxiliary metrics (`max_step`, `dx`, `lambda`, sine amplitude)
degraded as `nsg` increased; if anything `dx` peak and `lambda` range
got marginally better.

## Observations

- **Residual just meets the task bar.** `nsg = 0.20` brings the worst
  joint to exactly 2° off HOME, which matches the task's `≤ 2°`
  criterion. The smoke test's cosmetic verdict comparator uses strict
  `<`, so it still prints `FAIL residual=2.00°`; the underlying data
  (`|[99-100, 86-88, 91-93]| = [1, 2, 2]`) satisfies the task spec.
- **No oscillation or step-clamp stress introduced.** Sine max step
  stayed at 1°/tick across all three runs; the 2°/tick clamp only
  fired on phase-edge transients as in prior runs. So `nsg=0.20` does
  not pull hard enough to fight step-limits or inject noise.
- **EE tracking unchanged to slightly better.** Quiescent `err_target`
  dropped from 1.2 mm → 0.8 mm → ~0.5-1.0 mm across the three tests.
  Null-space projection does not conflict with the primary Cartesian
  task, so raising `nsg` helps posture without penalising tracking —
  consistent with theory and the head_assist_dls design comment.
- **Damping profile stable.** `manip` is effectively constant across
  runs because HOME is geometrically near-singular regardless of
  `nsg`. `lambda²` peak in test 3 is actually the lowest of the
  three (0.00615) — no saturation margin lost.
- **Direction correctness confirmed again.** Pitch ↔ Z targets, yaw
  ↔ Y target, signs and magnitudes consistent with tests 1 and 2.
- **`[HEAD-ASSIST] assistMode transition` log did not fire** for the
  same reason documented in test 2: the in-process
  `_last_assist_mode_flag` was still `"dls"` from the tail of the
  previous test, so `"dls" != "dls"` is false. Implementation is
  by-design.
- **Firmware quiet** — `STATUS:IDLE` throughout, no UART errors,
  post-test arm mechanically at `[100, 88, 93]`.

## Verdict
- **safe to continue tuning: yes.**
- **nullSpaceGain = 0.20: meets task criterion.** Residual = 2° on
  the equality boundary; EE tracking `≤ 1 mm`; clamp/oscillation
  headroom intact; direction correctness verified. **Hold this value
  as the new conservative baseline for the next phase of tuning.**

## Recommendation
1. **Lock `nullSpaceGain = 0.20` as the baseline.** It is the smallest
   value in the tested sweep that meets the task's 2° spec. Going
   higher now would be speculative — we do not yet have a reason to
   push further, and the monotone trend suggests that `0.25` might
   help but the marginal improvement is likely smaller than 1° (the
   slope is roughly `−1° per +0.05` and there is almost certainly
   a floor below which the near-singular HOME geometry cannot be
   compensated by null-space alone).
2. **Next test: begin a dedicated `gainM` sweep with `nsg` locked.**
   Move from `gainM=0.08` to `gainM=0.10` (the workstation default);
   everything else unchanged. This introduces perceptible EE motion
   for the operator without touching null-space.
3. **Fallback:** if in the `gainM` sweep the residual creeps back
   above 2° (because larger commanded motion imparts more momentum
   into near-singular configurations), try `nsg=0.22-0.25` to
   re-center.
4. **Do not touch** `lambdaMax`, `manipThresh`, `maxDqDegPerTick`,
   `maxDxMmPerTick` — they have comfortable headroom and are not
   the limiting factor at HOME.
5. Consider one speculative run at `nsg=0.25` later (still with
   `gainM=0.08`) to check whether the residual can be driven below
   2° without side effects; that probe would inform whether a
   non-conservative operator profile with stronger HOME pull is
   worth exposing.

## Raw Evidence

Smoke-test stdout (RESULTS block):
```text
captured 1605 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [96.0, 88.0, 93.0]  step_max=2.00°
return_center end BSG:  [99.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [102.0, 85.0, 89.0]  step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 89.0]
yaw_+8 end BSG:         [104.0, 85.0, 89.0]  step_max=2.00°
return_center3 end BSG: [100.0, 85.0, 89.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 91.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=4.00°
  [FAIL] arm returns to HOME after identity — residual=2.00°   ← cosmetic
                                                                  (< 2.0 strict)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Final routing_config readback:
```text
assistMode = rate
assistDls  = {'gainM': 0.25, 'lambdaMax': 0.1, 'manipThresh': 0.0007,
              'maxDqDegPerTick': 3.5, 'maxDxMmPerTick': 28,
              'nullSpaceGain': 0.18}   ← operator's stored tuning, untouched
```

Post-test live telemetry (after `SETPOSE HOME` recovery):
```text
BSG:   [100, 88, 93]
imu_q: [0.97229, 0.01068, 0.00317, -0.23352]
```
(Matches test 3 baseline to within static IMU noise; arm mechanically
at HOME.)
