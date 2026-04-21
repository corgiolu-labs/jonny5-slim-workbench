# DLS ASSIST Stress Test — Stage 0 (baseline confirmation)

First stage of the stress-test campaign planned in
`ai/reports/DLS_ASSIST_STRESS_TEST_PLAN_20260421_213009.md`.
Identical parameters to motion test 7
(`ai/reports/DLS_ASSIST_GAINM_TEST3_20260421_204213.md`), run against
the current Pi state (deploy `8ad990f` + runtime-fix `19f34ff`).

## Executive Summary
- stage: **0 (baseline confirmation)**
- result: **PARTIAL** — common stop criteria all green, but the
  final residual is `3°` instead of the `2°` observed in test 7. All
  other metrics reproduce test 7 closely. The 1° divergence on
  GOMITO at `final_identity` is within the common stop threshold
  (`residual ≤ 3°`) but exceeds the Stage 0 PASS criterion
  (`residual ≤ 2°`) defined in the stress-test plan.
- headline metrics:
  - pitch_down max excursion = **3°** (matches test 7)
  - `dx` peak per tick = **13.5 mm** (test 7: 13.2 mm; ≤ 15 mm cap)
  - `lambda²` min during pitch_down = **0.00840** (matches test 7;
    damping mitigation fully intact)
  - residual max at HOME = **3°** (test 7: 2°) ← deviation
  - `err_target` quiescent `0.6 – 1.0 mm`
  - `max_step` 2°/tick, sine max_step 1°/tick
- safety: no anomaly. STATUS:IDLE throughout, IMU valid at 125 Hz,
  assistMode restored to `rate`, arm recovered to HOME via post-test
  SETPOSE, smoke-test script restored from `.bak_stage0_*`.

## Environment
- commit (workstation): `9a38d1b` (stress-test plan).
- Pi `ws_server` PID **32418** (unchanged since 20:58 CEST deploy
  restart). Env `J5VR_TELEMETRY_FILE=/dev/shm/j5vr_telemetry.json`.
- Services `jonny5-ws-teleop`, `jonny5-spi-j5vr` active.
- WS client on 8557: `192.168.10.80:59280` (operator dashboard
  MSI.lan; unchanged).
- Arm at HOME at entry and exit.
- `routing_config.json` `assistMode="rate"` at entry and (via smoke
  `finally:`) exit. `assistDls` block untouched by this report
  (the smoke test injected its own baseline and restored).

## Safety Checks
- Physical routing_config backup (by hardened smoke test):
  `routing_config.json.bak_dls_smoke_20260421_213536`
- Pre-motion smoke-test source backup (in this stage):
  `verify_dls_assist_smoke.py.bak_stage0_20260421_213526`
- Smoke-test script restored from `.bak_stage0_*`; `py_compile`
  clean; `test_cfg["assistDls"]` literal back to committed hardened
  defaults after the run.
- No runtime-config source edits committed; the on-disk
  `routing_config.json` has `assistMode="rate"` post-test and
  `assistDls` still matches the validated baseline.
- Joint peaks during run: SPALLA 88° (at HOME), GOMITO 94° (+1°);
  BASE 107° during yaw. All inside hard limits.

## Motion Sequence
Same 9-phase sweep + SETPOSE recovery. Yaw amplitude capped at `+5°`.
**1 652 telemetry frames** captured.

## Telemetry / IMU — per-phase summary (with max excursion from HOME)

| phase | end_t | BSG end | imu_q_{w,x,y,z} | max excursion |
|---|---|---|---|---|
| identity_baseline | 1.50 | `[100.0, 88.0, 93.0]` | `[ 0.9736,  0.0110,  0.0064, -0.2278]` | 0° |
| pitch_down_5 | 3.51 | `[ 97.0, 88.0, 94.0]` | `[ 0.9754,  0.0107,  0.0069, -0.2202]` | **3°** |
| return_center | 5.03 | `[ 98.0, 87.0, 92.0]` | `[ 0.9705,  0.0115,  0.0078, -0.2407]` | 3° |
| pitch_up_neg5 | 7.04 | `[101.0, 86.0, 89.0]` | `[ 0.9673,  0.0082, -0.0125, -0.2533]` | 4° |
| return_center2 | 8.55 | `[100.0, 85.0, 90.0]` | `[ 0.9729,  0.0079, -0.0098, -0.2310]` | 4° |
| yaw_right_8 (+5°) | 10.56 | `[107.0, 86.0, 90.0]` | `[ 0.9828,  0.0071, -0.0081, -0.1840]` | 7° |
| return_center3 | 12.08 | `[100.0, 86.0, 90.0]` | `[ 0.9736,  0.0070, -0.0090, -0.2281]` | 7° |
| sine_pitch_4deg | 15.09 | `[ 99.0, 85.0, 91.0]` | `[ 0.9703,  0.0064, -0.0114, -0.2416]` | 4° |
| **final_identity** | 17.30 | `[ 99.0, 86.0, 90.0]` | `[ 0.9723,  0.0072, -0.0080, -0.2334]` | **3°** |

Post-test live telemetry (30 s after SETPOSE HOME):
`BSG=[100, 88, 93]`, `imu_q=[0.97333, 0.01074, 0.00659, -0.22900]`.

## DLS log — observed ranges
- `manip`: `0.00016 – 0.00024`
- `lambda²`: `0.00840 – 0.01012` (min during pitch_down **0.00840**,
  identical to test 7; mitigation fully active)
- `dx_cart_mm` per tick: `0.0 – 13.5` (peak slightly higher than
  test 7's 13.2 mm; 90% of the 15 mm cap — cap not exceeded)
- `err_target_mm`: `0.5 – 12.6` (transient); quiescent `~0.5 – 1.0`
- per-joint step clamp hit 2°/tick on phase edges only; sine
  max_step = 1°/tick
- `[HEAD-ASSIST] assistMode transition: None -> dls` fired at
  21:35:39 (PID 32418 restarted at 20:58, so `_last_assist_mode_flag`
  was reset to `None` — first mode=5 frame triggered the log as
  expected)

## Direct comparison vs Test 7 (same parameters)

| Metric | Test 7 (ref) | **Stage 0 (this)** | Δ |
|---|---|---|---|
| pitch_down end BSG | `[97, 88, 94]` | `[97, 88, 94]` | = |
| pitch_down max excursion | 3° | 3° | = |
| pitch_up end BSG | `[101, 86, 89]` | `[101, 86, 89]` | = |
| pitch_up excursion | 4° | 4° | = |
| yaw end BSG | `[107, 86, 90]` | `[107, 86, 90]` | = |
| yaw excursion | 7° | 7° | = |
| sine max_step | 1° | 1° | = |
| **final_identity end BSG** | `[99, 86, 91]` | **`[99, 86, 90]`** | **G −1°** |
| **residual max** | **2°** | **3°** | **+1°** |
| `max_step` | 2° | 2° | = |
| `dx` peak | 13.2 mm | 13.5 mm | +0.3 mm |
| `err_target` transient | 12.4 mm | 12.6 mm | +0.2 mm |
| `err_target` quiescent | 0.5-0.8 | 0.6-1.0 | ≈ = |
| `manip` range | 1.7-2.2 e-4 | 1.6-2.4 e-4 | ≈ = |
| `lambda²` min | 0.00840 | 0.00840 | = |
| assistMode final | rate | rate | = |

Seven out of eight structural metrics match test 7 byte-for-byte.
The only deviation is a single 1° offset on GOMITO at final_identity
(91° → 90°). Everything else — including the identical `lambda²`
minimum and the matching pitch_down end BSG — indicates the
mathematical behaviour of the DLS is unchanged; only the tail of the
return-to-HOME trajectory ended 1° further off HOME_q.

## Observations

- **The DLS behavior itself is unchanged.** Every mid-sweep phase
  reproduces test 7 exactly, including the damping profile
  (`lambda²` min `0.00840`) and the first-tick `dx` at pitch_down
  start (`~13 mm`). The mitigation (`manipThresh=1e-3`) is
  confirmed still active at runtime.
- **The residual discrepancy is localized to GOMITO.** The last few
  ticks before `final_identity` settled at
  `next_virt=[89.4, 91.4, 88.3]` which maps to physical
  `[99, 86, 90]`. In test 7 the equivalent ticks settled at
  `next_virt=[89.4, 91.4, 88.3]` → physical `[99, 86, 91]` — i.e.
  the same commanded virtual state, but GOMITO physical read back 1°
  lower. This is within the typical range of servo position noise
  and the null-space integration's dependency on the exact
  servo-feedback trajectory through the preceding sine phase.
- **Smoke-test verdict comparator now useful.** The Stage 0 run
  correctly tripped `FAIL residual=3.00°` under the fixed `<= 2.0`
  comparator — distinguishing this run from test 7 at the verdict
  level. Before the comparator fix, both runs would have printed
  the same (mis)leading "`FAIL residual=N.00°`" while differing
  in substance.
- **No safety regression.** `max_step`, sine `max_step`, joint
  limits, UART `STATUS:IDLE`, IMU validity, and `dx` peak all
  within their budgets. assistMode restored. SETPOSE recovery
  clean. No UART errors, no exceptions in the service journal.

## Verdict
- **Stage 0 PARTIAL.** The test did not violate any stop criterion,
  but it did not reproduce test 7's residual within the Stage 0
  tolerance (`≤ 2°`) set in the plan.

## Recommendation

**Do NOT proceed directly to Stage 1.** Two options, in decreasing
order of caution:

1. **Rerun Stage 0** (same parameters) to establish whether the 1°
   residual offset is deterministic (a real regression we need to
   triage) or run-to-run variance of the null-space recovery
   trajectory. The smoke test has async-driven timing (60 Hz
   inj, 2 Hz log) and SPI servo feedback jitter, so some
   ±1° variation in a single-joint final position is plausible.
   A second pass showing residual = 2° would reclassify Stage 0 as
   PASS; a second pass showing residual = 3° would confirm a real
   (small) regression that predates Stage 0.
2. **Accept 3° as the effective residual of the current deploy**,
   and proceed to Stage 1 with the relaxed PASS bar `residual ≤ 3°`.
   This is legitimate if the operator considers the 1° difference
   as run-to-run variance rather than regression. Stage 1 stop
   criteria would then track "Stage 0 baseline + 0°" rather than
   "test 7 value + 0°".

Strong preference for option (1): Stage 0 is cheap (one smoke test,
~35 s, zero cost on the robot state) and its whole purpose was
exactly to detect this kind of drift before the harder stages. A
single rerun is the cleanest way to decide.

**Common stop criteria and operator rules** remain unchanged for
any subsequent stage; no change to the plan document is proposed
on the basis of this single partial result.

## Raw Evidence

Smoke-test RESULTS block (Stage 0):
```text
captured 1652 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [97.0, 88.0, 94.0]   step_max=2.00°
return_center end BSG:  [98.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [101.0, 86.0, 89.0]  step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 90.0]
yaw_+8 end BSG:         [107.0, 86.0, 90.0]  step_max=2.00°
return_center3 end BSG: [100.0, 86.0, 90.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 90.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=3.00°
  [FAIL] arm returns to HOME after identity — residual=3.00°   ← real FAIL (comparator now <=2.0)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Key DLS log excerpts (hardening log + damping profile):
```text
21:35:39 [HEAD-ASSIST] assistMode transition: None -> dls        ← hardening log fires
21:35:39 [HEAD-ASSIST-DLS] target_mm=[60.0, 0.0, 321.0] cur=[90,90,90] manip=0.00022 lam2=0.00885 err=0.0 dx=0.0
21:35:42 target_mm=[59.4, 0.0, 307.9] cur=[90,90,90] manip=0.00022 lam2=0.00885 err=12.5 dx=13.1  ← pitch_down first tick
21:35:42 cur=[87,90,91] manip=0.00024 lam2=0.00840 err=12.3 dx=12.5                                ← lam² stays in plateau
21:35:45 target_mm=[59.4, 0.0, 334.1] cur=[88,91,87] manip=0.00017 lam2=0.00996 err=12.6 dx=13.5   ← dx peak
21:35:57 [HEAD-ASSIST] grace hold 17ms after raw deadman drop
```

Final routing_config readback:
```text
assistMode = rate
assistDls  = (baseline validata: gainM=0.15, lambdaMax=0.12,
              manipThresh=0.001, maxDqDegPerTick=2, maxDxMmPerTick=15,
              nullSpaceGain=0.2)   ← unchanged by this run
```

Post-test live telemetry (30 s after SETPOSE):
```text
BSG:   [100, 88, 93]
imu_q: [0.97333, 0.01074, 0.00659, -0.22900]
```
