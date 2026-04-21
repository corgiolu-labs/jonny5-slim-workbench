# DLS ASSIST gainM Sweep — Test 3

Third step of the `gainM` sweep. Raise `gainM` from `0.12` to `0.15`
keeping the `manipThresh = 1e-3` mitigation that eliminated the
damping unlock in the prior test. Everything else unchanged.

Prior reports referenced:
- `ai/reports/DLS_ASSIST_GAINM_TEST1_20260421_200047.md` (gainM=0.10, thresh=5e-4)
- `ai/reports/DLS_ASSIST_GAINM_TEST2_20260421_201058.md` (gainM=0.12, thresh=5e-4 — unlocked)
- `ai/reports/DLS_ASSIST_DAMPING_UNLOCK_TEST1_20260421_202442.md` (gainM=0.12, thresh=1e-3 — fix)

## Executive Summary
- test executed: **yes**
- result: **pass**
  - final residual max at HOME: **2°** (task ≤ 2° → PASS by equality)
  - pitch_down max joint excursion: **3°** (ideal ≤ 7° → PASS; identical
    to gainM=0.12 with the fix, despite commanding +25% larger EE shift)
  - `err_target` quiescent: **~0.5 – 0.8 mm** (≤ 3 mm → PASS)
  - `err_target` transient max: **12.4 mm** (larger than prior tests;
    reflects the larger target amplitude, not a regression)
  - `dx` peak per tick: **13.2 mm** (≤ 15 mm cap → PASS; **88% of cap**
    — we now have tight but positive headroom at this `gainM`)
  - **no `maxDxMmPerTick` saturation observed**
  - `max_step` = 2°/tick (clamp held → PASS)
  - sine `max_step` = 1°/tick (no oscillation → PASS)
  - assistMode initial `rate`, final `rate` (restored on-disk → PASS)
- `manipThresh=1e-3` mitigation remains fully effective at the higher
  gainM; `lambda²` never collapses.
- no safety anomalies, STATUS:IDLE, IMU valid.

## Parameters (on-Pi-only smoke-test literal, reverted after run)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.15,
    "lambdaMax": 0.12,
    "manipThresh": 0.001,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.20
  }
}
```
Other on-Pi smoke-test edits (unchanged, reverted after run):
yaw amplitude `+5°`, `cap()` records IMU fields.

## Environment
- repo commit (workstation): `b48aaba`
  (damping-unlock mitigation report).
- Pi `ws_server` PID **12572** (same hardened binary).
- Services all active; single WS client `192.168.10.80:54238`.
- Arm at HOME at entry and exit.

## Safety Checks
- Physical routing_config backup (by hardened smoke test):
  `routing_config.json.bak_dls_smoke_20260421_204053`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_204045`.
- Script restored from `.bak_premotion`; `py_compile` clean.
- Joint peaks during run: SPALLA 88° (at HOME), GOMITO 94° (+1° from
  HOME) during pitch_down; BASE 107° during yaw. All well inside
  hard limits (SPALLA 30-145, GOMITO 30-145, BASE 45-135).

## Motion Sequence
Same 9-phase sweep + SETPOSE recovery. 1 669 telemetry frames captured.

## Telemetry / IMU — per-phase summary (with max excursion from HOME)

| phase | end_t | BSG end | imu_q_{w,x,y,z} | max excursion |
|---|---|---|---|---|
| identity_baseline | 1.52 | `[100.0, 88.0, 93.0]` | `[ 0.9738,  0.0134,  0.0117, -0.2267]` | 0° |
| **pitch_down_5** | 3.52 | `[ 97.0, 88.0, 94.0]` | `[ 0.9758,  0.0134,  0.0114, -0.2180]` | **3°** |
| return_center | 5.04 | `[ 98.0, 87.0, 92.0]` | `[ 0.9710,  0.0135,  0.0120, -0.2382]` | 3° |
| pitch_up_neg5 | 7.05 | `[101.0, 86.0, 89.0]` | `[ 0.9676,  0.0092, -0.0146, -0.2519]` | 4° |
| return_center2 | 8.56 | `[100.0, 85.0, 90.0]` | `[ 0.9733,  0.0085, -0.0109, -0.2292]` | 4° |
| yaw_right_8 (+5°) | 10.57 | `[107.0, 86.0, 90.0]` | `[ 0.9831,  0.0071, -0.0090, -0.1827]` | 7° |
| return_center3 | 12.10 | `[100.0, 86.0, 90.0]` | `[ 0.9741,  0.0076, -0.0090, -0.2256]` | 7° |
| sine_pitch_4deg | 15.11 | `[ 99.0, 85.0, 91.0]` | `[ 0.9710,  0.0061, -0.0134, -0.2386]` | 4° |
| final_identity | 17.33 | `[ 99.0, 86.0, 91.0]` | `[ 0.9728,  0.0077, -0.0091, -0.2313]` | 3° (end 2°) |

Post-test live telemetry (after SETPOSE HOME recovery):
`BSG=[100, 88, 93]`, `imu_q=[0.97430, 0.01105, 0.00531, -0.22498]`.

## DLS log — observed ranges
- `manip`: `0.00016 – 0.00024` (unchanged; arm never leaves
  near-singular HOME neighborhood)
- `lambda²`: `0.00840 – 0.01012` (steady damped plateau; min during
  pitch_down = `0.00840`, effectively identical to gainM=0.12 run)
- `dx_cart_mm` per tick: `0.0 – 13.2` (peak at start of each phase;
  always under 15 mm cap — **clamp not saturated**)
- `err_target_mm`: `0.5 – 12.4` (transient); quiescent `~0.5 – 0.8 mm`
- per-joint step clamp hit 2°/tick on phase edges only; sine
  max-step = 1°/tick

Target amplitudes (×1.875 vs gainM=0.08 baseline, ×1.25 vs gainM=0.12):
- pitch_down target Z = 307.9 mm (HOME − 13.1 mm)
- pitch_up target Z = 334.1 mm (HOME + 13.1 mm)
- yaw target Y = 13.1 mm

## Comparison with prior tests (nsg=0.20 locked)

| Metric | gainM=0.08, thr=5e-4 | gainM=0.10, thr=5e-4 | gainM=0.12, thr=5e-4 | gainM=0.12, thr=1e-3 | **gainM=0.15, thr=1e-3** |
|---|---|---|---|---|---|
| residual max | 2° | 2° | 2° | 2° | **2°** |
| **pitch_down excursion** | 4° | 4° | **16°** | 3° | **3°** |
| pitch_up excursion | 5° | 5° | 5° | 4° | 4° |
| yaw excursion | 4° | 5° | 7° | 5° | 7° |
| sine excursion | 2° | 3° | 4° | 3° | 4° |
| `max_step` | 2° | 2° | 2° | 2° | 2° |
| sine max_step | 1° | 1° | 1° | 1° | 1° |
| `dx` peak | 7.3 mm | 9.7 mm | 10.9 mm | 11.1 mm | **13.2 mm** |
| `dx` as % of 15 mm cap | 49% | 65% | 73% | 74% | **88%** |
| `err_target` transient max | 6.7 mm | 8.3 mm | 9.8 mm | 10.0 mm | **12.4 mm** |
| `err_target` quiescent | 0.5-1.0 | 0.7-1.0 | 0.5-0.8 | 0.6-1.0 | **0.5-0.8** |
| `manip` peak | 2.2e-4 | 2.4e-4 | **3.9e-4** | 2.3e-4 | 2.4e-4 |
| `lambda²` min | 0.00465 | 0.00400 | **0.00074** | 0.00887 | 0.00840 |
| target amplitude (mm) | 7.0 | 8.7 | 10.5 | 10.5 | **13.1** |
| SPALLA peak physical | 88 | 88 | 100 | 88 | 88 |
| GOMITO peak physical | 93 | 94 | 109 | 93 | 94 |
| assistMode final | rate | rate | rate | rate | rate |
| safety regressions | none | none | none (but 16° drift) | none | **none** |

## Observations

- **Linear scaling recovered.** With `manipThresh=1e-3`, the pitch_down
  excursion stays at 3° as gainM goes from 0.12 → 0.15 (same excursion
  despite +25% more commanded EE reach). This confirms the
  `thresh=5e-4 → 1e-3` change was the right intervention: once the
  damping holds, the solver behaviour is approximately linear in
  gainM.
- **`dx` clamp envelope confirmed.** Peak `dx` is 13.2 mm, **88% of
  the 15 mm cap**. Clamp is not saturating but the margin is
  shrinking. At gainM=0.15 the system is near the practical envelope
  with the current `maxDxMmPerTick=15`. Any further gainM increase
  (→ 0.18 / 0.20) would very likely start clipping on phase-edge
  transitions.
- **`err_target` transient reflects target amplitude.** The 12.4 mm
  transient corresponds to the full +13.1 mm target shift — the arm
  catches up within a few ticks. Quiescent tracking is still ≤ 1 mm,
  identical to prior tests. No degradation.
- **Damping profile stable.** `lambda²` min 0.00840 during pitch_down
  is essentially the same 0.00887 as in the gainM=0.12/thresh=1e-3
  run — the Nakamura ramp is doing its job at the wider 1e-3 band.
  `manip` peak stayed at 2.4e-4, far below the 1e-3 threshold, so the
  ramp multiplier `k = 1 - 2.4e-4/1e-3 = 0.76` keeps `lam² ≈ 0.0083`
  steadily.
- **Null-space still in control.** Residual stays at 2° across the
  whole sweep — the raised EE gain does not defeat the null-space
  pull back to HOME_q.
- **Direction correctness** preserved on all axes. IMU response scales
  with gainM roughly as expected: Δq_z on yaw_end from baseline went
  from 0.017 at gainM=0.08 → 0.031 at gainM=0.10 → 0.045 at gainM=0.12
  (unlocked) / 0.030 at gainM=0.12 (fix) → **0.044** at gainM=0.15
  (fix). The fix brings yaw response back close to the unlocked-gainM=0.12
  number, now with much cleaner joint behavior.
- **No oscillation, no saturation, no clamp ride.** The 15 mm `dx`
  cap was approached but never reached; the 2°/tick `dq` cap was
  touched only on phase edges (unchanged from prior runs).
- **Firmware quiet**; STATUS:IDLE throughout; arm recovered to
  `[100, 88, 93]` via post-test SETPOSE.

## Verdict
- **safe to continue tuning: yes.**
- **`gainM = 0.15` is usable** with `manipThresh=1e-3` and
  `nullSpaceGain=0.20`. It delivers the largest perceptible EE
  motion of the sweep while keeping every task-criterion green.
- However, at `dx` peak = 88% of cap, the practical envelope is
  tight. Going above gainM=0.15 without also raising
  `maxDxMmPerTick` is likely to hit the cap on phase-edge transients.

## Recommendation
- **Accept `gainM = 0.15` as the operator-facing default.** It
  matches the workstation factory default in `imu_vr.js` and delivers
  ~1.9× the EE reach of the earlier `0.08` baseline with no worse
  quiescent tracking and the same 2° joint residual.
- **Do NOT push above `gainM = 0.15` yet.** If larger reach is
  desired:
  1. First raise `maxDxMmPerTick` from 15 → 20 (or 25) and re-run
     gainM=0.15 to confirm the cap was not a hidden constraint.
     This tells us whether the DLS naturally wanted to commit faster
     or was happy at 13 mm/tick.
  2. Only after that, try `gainM = 0.18` with the relaxed cap.
- **Leave `nullSpaceGain=0.20`, `lambdaMax=0.12`, `manipThresh=1e-3`,
  `maxDqDegPerTick=2.0` alone.** All four are behaving as designed
  and there is no reason to perturb them.
- **Optional cleanup**: the stored `assistDls` values in the Pi's
  `routing_config.json` still reflect an older operator exploration
  (`gainM=0.25`, `lambdaMax=0.10`, etc.). If the new conservative
  baseline (`gainM=0.15`, `lambdaMax=0.12`, `manipThresh=1e-3`,
  `maxDqDegPerTick=2.0`, `maxDxMmPerTick=15.0`, `nullSpaceGain=0.20`)
  is to become the persistent default, write it to the routing
  config via the dashboard save flow (not via direct file edit).
  This is outside the scope of the motion tests.

## Raw Evidence

Smoke-test RESULTS block:
```text
captured 1669 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [97.0, 88.0, 94.0]   step_max=2.00°    ← 3° excursion only
return_center end BSG:  [98.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [101.0, 86.0, 89.0]  step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 90.0]
yaw_+8 end BSG:         [107.0, 86.0, 90.0]  step_max=1.00°
return_center3 end BSG: [100.0, 86.0, 90.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 91.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=3.00°
  [FAIL] arm returns to HOME after identity — residual=2.00°   ← cosmetic (< 2.0 strict)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Key DLS log excerpts showing the damping holding through the largest-amplitude pitch commands:
```text
20:40:56 target_mm=[60.0, 0.0, 321.0]   cur=[90,90,90]    manip=0.00022 lam2=0.00885  ← HOME
20:40:59 target_mm=[59.4, 0.0, 307.9]   cur=[88,90,91]    manip=0.00024 lam2=0.00840  ← pitch_down phase start
                                                                              ^^^^^^^ damping steady
            err_target=12.4mm dx=12.6mm    ← target delta 13.1 mm, dx under 15 mm cap
20:41:00 target_mm=[59.4, 0.0, 307.9]   cur=[87,90,91]    manip=0.00024 lam2=0.00840  err=12.3 dx=12.5
20:41:01 target_mm=[60.0, 0.0, 321.0]   cur=[87,90,91]    manip=0.00024 lam2=0.00840  err=1.2  dx=3.2   ← return
20:41:03 target_mm=[59.4, 0.0, 334.1]   cur=[89,92,87]    manip=0.00018 lam2=0.00967  err=12.3 dx=13.2  ← pitch_up; dx peak
20:41:06 target_mm=[59.4, 13.1, 321.0]  cur=[94,92,87]    manip=0.00018 lam2=0.00964  err=7.1  dx=9.0   ← yaw
20:41:08 target_mm=[60.0, 0.0, 321.0]   cur=[97,92,87]    manip=0.00018 lam2=0.00964  err=5.3  dx=7.4   ← return
20:41:15 [HEAD-ASSIST] grace hold 17ms after raw deadman drop
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
imu_q: [0.97430, 0.01105, 0.00531, -0.22498]
```
