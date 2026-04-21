# DLS ASSIST Null-Space Tuning Test Report

Second controlled motion test, varying only `nullSpaceGain` relative
to the first motion test
(`ai/reports/DLS_ASSIST_MOTION_TEST_20260421_191712.md`) to isolate its
effect on the residual joint drift at HOME.

## Executive Summary
- test executed: **yes**
- result: **partial pass — measurable improvement**
- `nullSpaceGain`: `0.08` (test 1) → `0.15` (test 2)
- residual max joint offset at `final_identity` (HOME expected = [100, 88, 93]):
  - test 1: `|BSG_end - HOME| = [1, 4, 5]` → **max 5°**
  - test 2: `|BSG_end - HOME| = [2, 2, 3]` → **max 3°**  (↓ 40%)
- still above the task's `≤ 2°` pass threshold, but trend clearly
  confirms `nullSpaceGain` is the correct lever.
- no safety anomalies: no NaN, no explosion, `max_step = 2°/tick`,
  `STATUS:IDLE` throughout.
- assistMode initial: `rate` — assistMode final: `rate` (restored by
  smoke-test `finally:` block, verified on disk).

## Parameters Used (injected via modified smoke-test literals, Pi-only)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.08,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.15
  }
}
```

Other on-Pi smoke-test edits (same as test 1, reverted after run):
- yaw phase capped at `+5°` instead of `+8°`
- `cap()` extended to log `imu_q_{w,x,y,z}` + `imu_valid` per frame

No changes to committed files. Pi-side working copy reverted from
`verify_dls_assist_smoke.py.bak_premotion_20260421_192244` after the
run; `py_compile` clean.

## Environment
- repo commit (workstation): `2dc13a8`
  (`[REPORT] DLS ASSIST first motion test (partial pass)`).
- Pi `ws_server` PID still **12572** (same process as test 1 — same
  hardened binary, same env; `_last_assist_mode_flag` persisted in
  memory, see observations below).
- Services `jonny5-ws-teleop`, `jonny5-spi-j5vr`, HTTPS, MediaMTX all
  active throughout.
- WS client on 8557: `192.168.10.80:54238` (same operator dashboard,
  unchanged across both tests).
- Arm at HOME (`[100, 88, 93]`) at entry and exit.
- IMU live, `imu_valid=True`, rate in the 90-125 Hz band.

## Safety Checks
- `routing_config.json` backup (pre-existing from deploy prep):
  `routing_config.json.bak_pretest_20260421_185950`.
- Smoke-test physical backup created by the hardened script itself:
  `routing_config.json.bak_dls_smoke_20260421_192252`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_192244`.
- Post-test script state: restored from the `.bak_premotion` copy,
  compiled clean, parameters back to committed hardened values
  (`gainM=0.10`, etc.). No modified files committed to the repo.
- Final firmware status: `STATUS:IDLE` ~1 min after the test; arm
  physically back at `[100, 88, 93]` via the post-test `SETPOSE HOME`.

## Motion Sequence (executed in order)
1. identity_baseline 1.5 s
2. pitch_down_5 2.0 s (head pitch +5°)
3. return_center 1.5 s
4. pitch_up_neg5 2.0 s (head pitch −5°)
5. return_center2 1.5 s
6. yaw_right_8 2.0 s **(amplitude +5°, not +8° — reduced per task rule)**
7. return_center3 1.5 s
8. sine_pitch_4deg 3.0 s (±4°)
9. final_identity 2.0 s
10. post-test `SETPOSE 90 90 90 90 90 90 25 RTR5` (firmware recovery to HOME)

1 660 telemetry frames captured (vs 1 647 in test 1, same duration band).

## Telemetry / IMU — test 2 per-phase summary

| phase | end_t | BSG end | imu_q_{w,x,y,z} |
|---|---|---|---|
| identity_baseline | 1.51 | `[100.0, 88.0, 93.0]` | `[ 0.9717, 0.0085, -0.0049, -0.2358]` |
| pitch_down_5 | 3.52 | `[ 96.0, 88.0, 93.0]` | `[ 0.9717, 0.0079, -0.0051, -0.2358]` |
| return_center | 5.03 | `[ 99.0, 87.0, 92.0]` | `[ 0.9700, 0.0090, -0.0049, -0.2431]` |
| pitch_up_neg5 | 7.05 | `[102.0, 85.0, 88.0]` | `[ 0.9673, 0.0077, -0.0165, -0.2527]` |
| return_center2 | 8.57 | `[100.0, 85.0, 89.0]` | `[ 0.9724, 0.0059, -0.0168, -0.2327]` |
| yaw_right_8 (+5°) | 10.59 | `[104.0, 85.0, 89.0]` | `[ 0.9767, 0.0052, -0.0156, -0.2138]` |
| return_center3 | 12.10 | `[100.0, 85.0, 89.0]` | `[ 0.9724, 0.0062, -0.0170, -0.2328]` |
| sine_pitch_4deg | 15.12 | `[ 99.0, 85.0, 90.0]` | `[ 0.9694, 0.0080, -0.0103, -0.2449]` |
| final_identity | 17.33 | `[ 98.0, 86.0, 90.0]` | `[ 0.9707, 0.0076, -0.0098, -0.2399]` |

Post-test telemetry (30 s after `SETPOSE HOME` recovery):
`BSG=[100, 88, 93]`, `imu_q=[0.97241, 0.00977, 0.00031, -0.23309]`.

## DLS log — test 2 (ranges)
From journalctl entries `[HEAD-ASSIST-DLS]` during the test window:
- `manip`: `0.00015 – 0.00023` (threshold `5e-4`, damping always active at HOME, as expected)
- `lambda²`: `0.00419 – 0.00688` (peak below `lambdaMax² = 0.0144` — no saturation)
- `dx_cart_mm` per tick: `0.0 – 7.2` (well under `maxDxMmPerTick=15` cap)
- `err_target_mm`: `0.6 – 6.6` (transient); quiescent `~0.8 mm`
- per-joint step: clamp `maxDqDegPerTick=2.0` reached during phase transitions,
  never exceeded; sine phase `max_step=1°` (no amplification of oscillation)

## Side-by-side comparison (test 1 vs test 2)

| Metric | Test 1 (`nsg=0.08`) | Test 2 (`nsg=0.15`) | Δ |
|---|---|---|---|
| `final_identity` BSG | `[99, 84, 88]` | `[98, 86, 90]` | — |
| residual per joint `|ΔBSG|` | `[1, 4, 5]` | `[2, 2, 3]` | S, G ↓; B +1 |
| **residual max** | **5°** | **3°** | **↓ 40%** |
| smoke-test "returns to HOME" verdict | FAIL 5° | FAIL 3° | still FAIL (<2° target) |
| `max_step` (clamp reached but not exceeded) | 2° | 2° | = |
| `sine_pitch_4deg` max_step | 1° | 1° | = |
| `err_target` transient max | ~7.4 mm | ~6.6 mm | ↓ slightly |
| `err_target` quiescent | ~1.2 mm | ~0.8 mm | ↓ slightly |
| `manip` range | 0.00015 – 0.00022 | 0.00015 – 0.00023 | ≈ |
| `lambda²` range | 0.00465 – 0.00718 | 0.00419 – 0.00688 | ≈ |
| `dx` peak per tick | 8.8 mm | 7.2 mm | ↓ slightly |
| direction correctness all 3 axes | OK | OK | = |
| safety regressions | none | none | = |
| `assistMode transition` log in journal | `None -> dls` at start | **not emitted** (see obs. below) | — |

## Observations

- **Null-space drift reduced as expected.** Raising `nullSpaceGain`
  from 0.08 to 0.15 cuts the max residual at HOME from 5° to 3°
  (−40%). Shoulder residual halved (4°→2°), elbow residual improved
  (5°→3°). Base residual grew by 1° — within noise given the DLS is
  near-singular at HOME and the solver redistributes dq across joints.
- **No oscillation introduced.** The sine phase max-step stayed at
  1°/tick; there is no sign of null-space bias feedback amplifying
  step-to-step oscillation at this gain level. The headroom to
  `maxDqDegPerTick=2°` was never fully consumed by null-space + primary
  task combined.
- **Cartesian tracking improved slightly.** `err_target` transient and
  quiescent both got a little smaller — consistent with the expectation
  that null-space projection does not fight the primary task (it
  projects into the null-space of J by construction), so raising
  null-space gain mainly helps joint posture without hurting EE track.
- **Damping profile identical to test 1.** `manip` still pinned in
  `1.5e-4 – 2.3e-4` (HOME is geometrically singular for this 3-DoF
  position IK — the Yoshikawa measure is tiny regardless of
  `nullSpaceGain`). `lambda²` stays within its safe band, never
  saturating at `lambdaMax²`.
- **Direction correctness reconfirmed.** All three head-motion axes
  produced the expected EE target shift direction (pitch ↔ Z axis,
  yaw ↔ Y axis). IMU quaternion deltas move in the expected sign.
  Example: `yaw_right +5°` → target_mm = `[59.7, 7.0, 321]`, BSG base
  increased from 100 → 104, IMU `q_z` magnitude decreased from
  `-0.2327` to `-0.2138` (consistent direction when converting the
  IMU frame to base-referenced yaw).
- **`assistMode transition` log did NOT fire at test 2 start.** The
  hardening's in-process flag `_last_assist_mode_flag` was left at
  `"dls"` from the tail of test 1 (the smoke client kept sending
  mode=5 frames during grip release while config was still `dls`;
  then disconnected before any `rate` frames could flip the flag).
  Between tests the dashboard client does not send mode=5 frames,
  so the flag did not reset. When test 2's first mode=5 frame landed
  with `assist_mode_flag = "dls"`, the comparison `"dls" != "dls"` is
  false — no transition log. The implementation is behaving as
  designed (log fires only on actual transitions); this is worth
  keeping in mind for future audit-trail expectations.
- **Firmware quiet.** No UART errors, `STATUS:IDLE` steady before and
  after, no stuck servos, no unexpected warnings in `journalctl`.

## Verdict
- **safe to continue tuning: yes.**
- **direction correctness:** OK on all three axes.
- **`nullSpaceGain=0.15`:** **keep for now as a candidate**, but it is
  not sufficient on its own to meet the task's `≤ 2°` residual bar —
  so the primary recommendation is to try **`nullSpaceGain=0.20`**
  next (see below), with all other parameters unchanged.
- **err_target final:** quiescent `~0.8 mm`, well under the task's
  `≤ 3 mm` success criterion → PASS on EE tracking.
- **max_step:** `2°/tick` (clamp triggers but does not exceed) → PASS
  on step-cap criterion.
- **final residual:** `3°` → does not meet `≤ 2°` → PARTIAL on that
  specific criterion.
- **assistMode final:** `rate` → PASS on post-test state criterion.

## Recommendation
1. **Run test 3 with `nullSpaceGain=0.20`**, everything else identical
   to test 2. Expected outcome: residual below the 2° bar while still
   staying clear of oscillation and without hurting EE tracking. If
   met, lock `nullSpaceGain=0.20` as the new conservative baseline.
2. If `0.20` overshoots and introduces oscillation or step-cap
   saturation, back off to `0.17-0.18` and redo the sweep.
3. Only after null-space is within spec, start varying `gainM`
   upward (`0.10`, then `0.15`) to increase operator perceptibility;
   keep `nullSpaceGain` locked in that sweep to avoid confounding.
4. Do **not** touch `lambdaMax`, `manipThresh`, `maxDqDegPerTick`,
   `maxDxMmPerTick` yet — they have comfortable headroom and are not
   the current limiting factor.

## Raw Evidence

Smoke-test final state echo (stdout):
```text
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [96.0, 88.0, 93.0]  step_max=2.00°
return_center end BSG:  [99.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [102.0, 85.0, 88.0]  step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 89.0]
yaw_+8 end BSG:         [104.0, 85.0, 89.0]  step_max=2.00°
return_center3 end BSG: [100.0, 85.0, 89.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [98.0, 86.0, 90.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=4.00°
  [FAIL] arm returns to HOME after identity — residual=3.00°
  [OK]   no joint step >5°/frame — max_step=2.00°
```
(The `yaw_+8` label name is cosmetic; the injected head quaternion was
`q_yaw(+5)`, as patched on the Pi before the run.)

Final routing_config readback:
```text
assistMode = rate
assistDls  = {'gainM': 0.25, 'lambdaMax': 0.1, 'manipThresh': 0.0007,
              'maxDqDegPerTick': 3.5, 'maxDxMmPerTick': 28,
              'nullSpaceGain': 0.18}   ← operator's stored values,
                                          preserved untouched
```

Post-test live telemetry (30 s after recovery):
```text
BSG:   [100, 88, 93]
imu_q: [0.97241, 0.00977, 0.00031, -0.23309]
```
(Matches the test 2 baseline to within static IMU noise; arm is
mechanically back at HOME.)
