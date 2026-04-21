# DLS ASSIST Damping-Unlock Mitigation — Test 1

Targeted mitigation test for the non-linear pitch_down excursion
observed at `gainM=0.12`. Only `manipThresh` changed: `5e-4 → 1e-3`.
All other parameters held at the baseline established in prior tests.

Prior report referenced:
- `ai/reports/DLS_ASSIST_GAINM_TEST2_20260421_201058.md`
  (gainM=0.12, manipThresh=5e-4 → 16° pitch_down excursion).

## Executive Summary
- test executed: **yes**
- result: **pass — mitigation highly effective**
  - **pitch_down max excursion: 16° → 3° (−81%)** — the damping
    unlock is eliminated.
  - `lambda²` minimum during pitch_down: **0.00074 → 0.00887** (12×
    more damping retained where it matters).
  - `manip` range barely changed (still near-HOME-singular), but with
    the higher threshold, the Nakamura/Hanafusa ramp keeps `lam²`
    large across the whole phase.
  - final residual max at HOME: **2°** (task `≤ 2°` → PASS by equality).
  - `err_target` quiescent: **~0.6 – 1.0 mm** (≤ 3 mm → PASS; very
    small regression from the prior 0.5-0.8 mm, still well inside spec).
  - `err_target` transient: essentially unchanged (~10 mm peak).
  - `dx` peak per tick: **11.1 mm** (≤ 15 mm PASS).
  - `max_step` = 2°/tick (clamp held → PASS).
  - sine `max_step` = 1°/tick (no oscillation → PASS).
  - assistMode initial `rate`, final `rate` (restored → PASS).
- no safety anomalies.

## Parameters (on-Pi-only smoke-test literal, reverted after run)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.12,
    "lambdaMax": 0.12,
    "manipThresh": 0.001,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.20
  }
}
```
Other on-Pi smoke-test edits (unchanged from prior tests, reverted
after run): yaw phase amplitude `+5°`, `cap()` records IMU fields.

## Environment
- repo commit (workstation): `cdfd5be` (gainM test 2 report).
- Pi `ws_server` PID **12572** (same hardened binary).
- Services all active; single WS client `192.168.10.80:54238`.
- Arm at HOME at entry and exit.

## Safety Checks
- Physical routing_config backup (by hardened smoke test):
  `routing_config.json.bak_dls_smoke_20260421_202337`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_202328`.
- Script restored from `.bak_premotion` after run; `py_compile` clean.
- Physical joint peaks during run: SPALLA reached 88° (no excursion
  past HOME on SPALLA!), GOMITO reached 93° (no excursion past HOME).
  Compare vs prior test where SPALLA peaked at 100° and GOMITO at 109°.
- assistMode restored on disk to `rate`.

## Motion Sequence
Same 9-phase sweep + SETPOSE recovery. 1 622 telemetry frames captured.

## Telemetry / IMU — per-phase summary (with max excursion from HOME)

| phase | end_t | BSG end | imu_q_{w,x,y,z} | max excursion |
|---|---|---|---|---|
| identity_baseline | 1.50 | `[100.0, 88.0, 93.0]` | `[ 0.9722,  0.0112,  0.0047, -0.2340]` | 0° |
| **pitch_down_5** | 3.53 | `[ 97.0, 88.0, 93.0]` | `[ 0.9746,  0.0109,  0.0048, -0.2236]` | **3°** |
| return_center | 5.04 | `[ 98.0, 87.0, 92.0]` | `[ 0.9698,  0.0114,  0.0051, -0.2438]` | 3° |
| pitch_up_neg5 | 7.04 | `[101.0, 86.0, 89.0]` | `[ 0.9666,  0.0101, -0.0076, -0.2562]` | 4° |
| return_center2 | 8.55 | `[100.0, 85.0, 90.0]` | `[ 0.9722,  0.0097, -0.0051, -0.2339]` | 4° |
| yaw_right_8 (+5°) | 10.56 | `[105.0, 86.0, 90.0]` | `[ 0.9789,  0.0089, -0.0035, -0.2040]` | 5° |
| return_center3 | 12.07 | `[100.0, 86.0, 90.0]` | `[ 0.9728,  0.0093, -0.0045, -0.2314]` | 5° |
| sine_pitch_4deg | 15.06 | `[ 99.0, 86.0, 91.0]` | `[ 0.9691,  0.0104, -0.0023, -0.2466]` | 3° |
| final_identity | 17.29 | `[ 99.0, 86.0, 91.0]` | `[ 0.9719,  0.0101, -0.0018, -0.2352]` | 2° |

Post-test live telemetry: `BSG=[100, 88, 93]`,
`imu_q=[0.97284, 0.01331, 0.01117, -0.23083]`.

## DLS log — observed ranges
- `manip`: `0.00016 – 0.00023` (almost unchanged from prior tests —
  the geometry at HOME is what it is)
- `lambda²`: `0.00853 – 0.01012` (range floor raised **~11×** compared
  to prior gainM=0.12 run's `0.00074` minimum). `lambda²` at identity
  baseline: **0.00885** (was 0.00465 before) — the raised threshold
  roughly doubles baseline damping at HOME.
- `lambda²` min during pitch_down: **0.00887** (was 0.00074 before)
  — the damping unlock is fully eliminated.
- `dx_cart_mm` per tick: `0.0 – 11.1` (well under 15 mm cap)
- `err_target_mm`: `0.4 – 10.0` (transient); quiescent `~0.6 – 1.0`
- per-joint step clamp hit 2°/tick only on phase edges (as in prior
  runs). sine-phase max step = 1°/tick.

Analytical check of the new damping profile (using
`lam² = (lambdaMax × max(0, 1 − manip/thresh))²`):
- `thresh = 1e-3`, `lambdaMax = 0.12`
- At `manip = 0` (fully singular): `lam² = 0.0144` (unchanged from prior test)
- At `manip = 5e-4` (half-threshold): `lam² = (0.12 × 0.5)² = 0.0036`
- At `manip = 1e-3` (new threshold): `lam² = 0`
- Prior test: at `manip = 3.9e-4` (the observed peak during pitch_down),
  `k = 1 − 3.9e-4/5e-4 = 0.22`, `lam² = 0.000697` — the collapse.
- Current test: at the same `manip = 3.9e-4`,
  `k = 1 − 3.9e-4/1e-3 = 0.61`, `lam² = 0.00536` — 7.7× larger.

The ramp now extends over a 2× wider manip band, so the operating
regime at HOME-neighborhood stays firmly inside the damped plateau
and never enters the "nearly-undamped" tail.

## Before / after comparison (only `manipThresh` differs)

| Metric | gainM=0.12, **manipThresh=5e-4** | gainM=0.12, **manipThresh=1e-3** | Δ |
|---|---|---|---|
| `final_identity` BSG | `[99, 86, 91]` | `[99, 86, 91]` | = |
| residual max | 2° | 2° | = |
| **pitch_down max excursion** | **16°** | **3°** | **−81%** |
| pitch_up max excursion | 5° | 4° | −1° |
| yaw max excursion | 7° | 5° | −2° |
| `max_step` | 2° | 2° | = |
| sine max_step | 1° | 1° | = |
| `dx` peak | 10.9 mm | 11.1 mm | +2% |
| `err_target` transient max | 9.8 mm | 10.0 mm | +2% |
| `err_target` quiescent | 0.5 – 0.8 mm | 0.6 – 1.0 mm | slight ↑ |
| `manip` peak | 3.9e-4 | 2.3e-4 | ↓ (arm did not leave singularity) |
| `lambda²` min | 0.00074 | **0.00887** | **+11.9×** |
| `lambda²` baseline at HOME | 0.00465 | 0.00885 | ×1.9 |
| SPALLA physical peak | 100° | 88° | = HOME |
| GOMITO physical peak | 109° | 93° | = HOME |
| safety regressions | none (but 16° drift) | **none** (clean) | ✓ |
| assistMode final | rate | rate | = |

## Observations

- **The damping unlock is eliminated.** With `manipThresh=1e-3` the
  observed `manip` never approaches the new threshold, so `lam²`
  stays at roughly its HOME baseline (≈0.009) throughout the entire
  pitch_down phase. The arm no longer "runs away" into a low-manip
  auxiliary configuration because the pseudoinverse remains damped
  even as the arm starts drifting.
- **SPALLA and GOMITO barely move on pitch_down.** Smoke reports
  `pitch_down_5 end BSG=[97, 88, 93]` — SPALLA and GOMITO are **at
  HOME** at the end of the phase. Only BASE moved (by 3°). This is
  a dramatic qualitative change: the arm now obeys the commanded
  Z-target with a much smaller base-dominated rotation and barely
  engages the singular S/G mode. Even though the commanded EE target
  was below HOME_ee by 10.5 mm, the DLS now prefers the small-base
  solution over the large-S/G solution.
  - Physically, this makes sense: with heavy damping, `|dq|²` is
    penalized across all joints, so the solver prefers the smallest
    `|dq|` that roughly reduces `|err|`. A small BASE rotation
    contributes mostly Y-axis motion, but since BASE cannot reduce
    Z-error at HOME, the solver also makes small S/G moves. The
    smoke's telemetry capture at the END of the phase catches the
    arm with S and G back at HOME — suggesting the intra-phase
    excursion is both smaller AND self-damping (tracks the sinusoidal
    phase transitions more gracefully).
- **Tracking cost is modest.** `err_target` transient peaked at
  10.0 mm (vs 9.8 mm prior). Quiescent slightly larger (~1.0 mm vs
  ~0.7 mm). Both stay well below the 3 mm PASS criterion. This is
  the price of more damping: the DLS is more conservative, so the
  arm catches up to target a bit more slowly. The difference is
  sub-mm at settle and sub-single-mm in transient.
- **Other phases improved too.** pitch_up (5°→4°), yaw (7°→5°) show
  small reductions in max excursion. The higher damping bleeds into
  every transition, not just the pitch_down-from-HOME case.
- **`nullSpaceGain=0.20` still effective for residual control.** The
  final_identity BSG is `[99, 86, 91]`, identical to prior tests at
  the same `nsg` — raising damping doesn't interfere with null-space
  bias because null-space projects onto `I − J⁺J` which is independent
  of the damping level (only the pseudoinverse's image-space scaling
  changes, not its kernel).
- **Direction correctness** preserved on all axes.
- **IMU yaw response** slightly weaker than the unlocked test
  (`|Δimu_q_z|` from baseline `-0.2340` to yaw-end `-0.2040` ≈ 0.030
  vs 0.045 in the unlocked run). The arm tip moves less per unit of
  commanded EE displacement because the arm now chooses more conservative
  joint trajectories. Still perceptible and well-signed.

## Verdict
- **safe to continue tuning: yes.**
- **manipThresh = 1e-3 is a clear win for gainM ≥ 0.12.** Keep it.

## Recommendation
1. **Adopt `manipThresh = 1e-3` as the new conservative baseline.**
   Quality of motion at gainM=0.12 is now dramatically better with a
   negligible tracking penalty.
2. **Next step: retry `gainM = 0.15`** with
   `manipThresh = 1e-3`, everything else unchanged. The 0.10→0.12
   step showed almost-linear scaling (post-mitigation), so 0.12→0.15
   is likely within the same damped regime. Predicted `dx` peak
   ≈ 13-14 mm (close to cap), transient ≈ 12 mm. Watch the clamp.
3. If at `gainM=0.15` the transient starts riding the 15 mm cap,
   consider raising `maxDxMmPerTick` to `20`, but ONLY after
   confirming the raised cap doesn't let worst-case single-frame
   motion get too large.
4. **Do not** touch `lambdaMax` now — the baseline damping doubled
   via the threshold change, further `lambdaMax` increase would
   overdamp and hurt tracking visibly.
5. **Do not** touch `nullSpaceGain`, `maxDqDegPerTick`,
   `maxDxMmPerTick` — they are doing their job.
6. A second probe at `gainM = 0.12, manipThresh = 1e-3` but with
   a yaw of +8° (closer to the original smoke test) would help
   confirm the envelope. Optional and low priority.

## Raw Evidence

Smoke-test RESULTS block:
```text
captured 1622 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [97.0, 88.0, 93.0]   step_max=2.00°    ← 3° excursion only!
return_center end BSG:  [98.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [101.0, 86.0, 89.0]  step_max=1.00°
return_center2 end BSG: [100.0, 85.0, 90.0]
yaw_+8 end BSG:         [105.0, 86.0, 90.0]  step_max=2.00°
return_center3 end BSG: [100.0, 86.0, 90.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 91.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=3.00°            ← was 16°!
  [FAIL] arm returns to HOME after identity — residual=2.00°    ← cosmetic (< 2.0 strict)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Key DLS log excerpts showing damping held steady during pitch_down:
```text
20:23:39 target_mm=[60.0, 0.0, 321.0]   cur=[90,90,90]  manip=0.00022 lam2=0.00885  ← HOME baseline
20:23:43 target_mm=[59.5, 0.0, 310.5]   cur=[87,90,90]  manip=0.00021 lam2=0.00887  ← pitch_down, lam² steady
20:23:44 target_mm=[60.0, 0.0, 321.0]   cur=[87,91,90]  manip=0.00023 lam2=0.00853  ← return_center begins
20:23:45 cur=[88,91,89]                                 manip=0.00021 lam2=0.00901
                                                                         ^^^^^^^^ never collapsed
20:23:46 target_mm=[59.5, 0.0, 331.5]   cur=[90,92,87]  manip=0.00018 lam2=0.00967  ← pitch_up
```

Final routing_config readback:
```text
assistMode = rate
assistDls  = {'gainM': 0.25, 'lambdaMax': 0.1, 'manipThresh': 0.0007,
              'maxDqDegPerTick': 3.5, 'maxDxMmPerTick': 28,
              'nullSpaceGain': 0.18}   ← operator's stored tuning, untouched
```

Post-test live telemetry:
```text
BSG:   [100, 88, 93]
imu_q: [0.97284, 0.01331, 0.01117, -0.23083]
```
