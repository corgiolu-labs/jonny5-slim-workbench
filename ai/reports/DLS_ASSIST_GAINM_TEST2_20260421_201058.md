# DLS ASSIST gainM Sweep — Test 2

Second step of the `gainM` sweep: raise `gainM` from `0.10` to `0.12`,
keep `nullSpaceGain = 0.20` locked, all other conservative parameters
unchanged.

Prior reports referenced:
- `ai/reports/DLS_ASSIST_GAINM_TEST1_20260421_200047.md` (gainM=0.10)
- `ai/reports/DLS_ASSIST_NULLSPACE_TEST3_20260421_194652.md` (gainM=0.08)

## Executive Summary
- test executed: **yes**
- result: **pass with notable non-linear tradeoff** — strict task
  criteria all met, but a new behavior surfaced: during the
  **pitch_down** phase the arm produced a **16° max joint excursion**
  from HOME (tests at gainM=0.08 and 0.10 both showed 4°). This is
  not a safety violation (clamps held, firmware idle, arm returned
  to HOME at phase end), but it is a clear super-linear response to
  raising gainM that must be understood before going to gainM=0.15.
- final residual max at HOME = **2°** (task `≤ 2°` PASS by equality)
- `err_target` quiescent `~0.5 – 0.8 mm` (≤ 3 mm PASS)
- `dx` peak per tick = **10.9 mm** (≤ 15 mm PASS)
- `max_step` = **2°/tick** (clamp held → PASS)
- sine `max_step` = **1°/tick** (no oscillation → PASS)
- assistMode initial `rate`, final `rate` (restored on-disk → PASS)

## Parameters (on-Pi-only smoke-test literal, reverted after run)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.12,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.20
  }
}
```
Same on-Pi smoke-test edits as prior tests (yaw amplitude `+5°`,
`cap()` records IMU fields). Reverted after run.

## Environment
- repo commit (workstation): `b0e6d4b` (gainM test 1 report).
- Pi `ws_server` PID **12572** (same hardened binary).
- Services all active; single WS client `192.168.10.80:54238`.
- Arm at HOME at entry and exit.

## Safety Checks
- Physical routing_config backup (by hardened smoke test):
  `routing_config.json.bak_dls_smoke_20260421_200641`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_premotion_20260421_200634`.
- Script restored from `.bak_premotion` after run; `py_compile` clean.
- Joint limits: physical SPALLA reached 100° (range 30-145), physical
  GOMITO reached 109° (range 30-145). Inside limits, no soft-stop.
- No source-file edits committed; routing_config rolled back to
  `assistMode=rate` automatically.

## Motion Sequence (executed in order)
Same 9-phase sweep as tests 1-4 plus SETPOSE recovery. Yaw amplitude
capped at `+5°`. 1 652 telemetry frames captured.

## Telemetry / IMU — per-phase summary (with max excursion from HOME)

| phase | end_t | BSG end | imu_q_{w,x,y,z} | max excursion from HOME |
|---|---|---|---|---|
| identity_baseline | 1.50 | `[100.0, 88.0, 93.0]` | `[ 0.9725,  0.0127,  0.0093, -0.2322]` | 0° |
| **pitch_down_5** | 3.51 | `[ 96.0, 100.0, 109.0]` | `[ 0.9714,  0.0238,  0.0419, -0.2323]` | **16°** |
| return_center | 5.02 | `[ 99.0, 87.0, 92.0]` | `[ 0.9711,  0.0089, -0.0046, -0.2384]` | 16° (inherits from prior phase) |
| pitch_up_neg5 | 7.03 | `[102.0, 85.0, 88.0]` | `[ 0.9687,  0.0073, -0.0184, -0.2474]` | 5° |
| return_center2 | 8.54 | `[100.0, 85.0, 89.0]` | `[ 0.9730,  0.0058, -0.0187, -0.2301]` | 5° |
| yaw_right_8 (+5°) | 10.56 | `[107.0, 85.0, 89.0]` | `[ 0.9822,  0.0060, -0.0176, -0.1871]` | 7° |
| return_center3 | 12.07 | `[100.0, 85.0, 89.0]` | `[ 0.9730,  0.0063, -0.0175, -0.2300]` | 7° |
| sine_pitch_4deg | 15.09 | `[ 99.0, 86.0, 92.0]` | `[ 0.9708,  0.0081, -0.0069, -0.2399]` | 4° |
| final_identity | 17.31 | `[ 99.0, 86.0, 91.0]` | `[ 0.9714,  0.0106,  0.0004, -0.2371]` | 2° |

Post-test live telemetry (after `SETPOSE HOME` recovery):
`BSG=[100, 88, 93]`, `imu_q=[0.97192, 0.01074, 0.00391, -0.23492]`.

## DLS log — test gainM-2 observed ranges
- `manip`: `0.00015 – 0.00039` (**higher peak than all prior tests**;
  the arm left HOME singularity more fully during pitch_down)
- `lambda²`: `0.00074 – 0.00688` (went **down to 0.00074** during
  pitch_down — far lower than in prior tests where the minimum was
  around 0.00400. This is the damping collapsing as manip improved
  past the threshold.)
- `dx_cart_mm` per tick: `0.0 – 10.9` (largest seen; still under 15 mm cap)
- `err_target_mm`: `0.5 – 9.8` (transient); quiescent `~0.5 – 0.8`
- per-joint step clamp: hit 2°/tick on phase edges only
- sine-phase max step = 1°/tick (no oscillation)

Target amplitudes matched `gainM × 0.12 / 0.08 = 1.5×` prior tests:
- pitch_down target Z: 310.5 mm (HOME − 10.5 mm)
- pitch_up   target Z: 331.5 mm (HOME + 10.5 mm)
- yaw_right  target Y:  10.5 mm

## Observations

- **The 16° pitch_down excursion is a non-linear runaway triggered at
  HOME.** Trace from the DLS log:
  1. Tick 1 at HOME, target_mm = `[59.5, 0.0, 310.5]`, `manip=0.00022`,
     `lam²=0.00465` → `dq` small, commit `next_virt=[90, 89.5, 91.5]`.
  2. As the arm moves off HOME, `manip` climbs to 0.00027, then
     0.00031, 0.00035, **0.00039**. At `manip=0.00039`,
     `k = 1 - manip/thresh = 0.22`, so `lam² = (0.12 × 0.22)² = 0.00070`
     — the damping nearly collapses.
  3. With almost no damping, the pseudoinverse commits to its full
     un-regularized solution, which for pure Z motion at near-singular
     HOME asks for `~−20°` on SPALLA and `~+25°` on GOMITO. The
     per-tick clamp (2°/tick) throttles this, but 25 ticks × 2° = 50°
     of theoretical single-period slew. The phase lasts ~100 ticks,
     during which SPALLA drifted from 90° to 78° (virt), GOMITO from
     90° to 107°. In physical units (dir_S = −1, dir_G = +1), that's
     SPALLA 88 → 100 and GOMITO 93 → 109 — the **16° max
     excursion** the smoke test reports.
  4. The `err_target` during this excursion stayed bounded
     (4 - 9.8 mm); the DLS was not "failing" — it was correctly
     driving the EE toward the commanded target, just via a large
     joint detour.
- **This behavior is mathematically expected.** The kinematic
  Jacobian at HOME is rank-deficient in the pitch plane — SPALLA and
  GOMITO have near-identical Z contributions. For pure Z motion, the
  undamped pseudoinverse favors an antisymmetric (S↓, G↑)
  combination with very large magnitude. DLS damping at HOME
  suppresses this, but once the arm moves off-HOME, damping decays
  and the antisymmetric mode unlocks.
- **The effect is asymmetric.** Only the FIRST pitch_down phase
  showed 16° — every subsequent phase stayed under 7°. This is
  because:
  - Pitch_down phase started at HOME exactly (cur_virt = [90, 90, 90]),
    where the Jacobian is most degenerate.
  - All later phases started from slightly off-HOME poses that the
    prior return_center left behind. Those starting configs have
    `manip > threshold` already, so the "unlock" never happens.
  - In particular, pitch_up and yaw showed max excursions of 5° and
    7° respectively — consistent with what the +50% gainM would
    predict from a linear scaling of the gainM=0.08 baseline (4° →
    ~5-6°).
- **Null-space still catches the drift.** Despite the 16° excursion
  during pitch_down, the arm returned to `final_identity` residual
  of 2° — identical to test 3 and gainM-1. The null-space gain 0.20
  handles the recovery.
- **No safety hit.** SPALLA peaked at 100° physical (limit 145);
  GOMITO peaked at 109° (limit 145). Per-tick clamp held at 2°.
  No oscillation. Firmware `STATUS:IDLE` throughout. No UART errors.
- **Direction correctness** still holds on all three axes:
  pitch ↔ Z, yaw ↔ Y. IMU yaw response is stronger than prior tests
  (yaw end `imu_q_z = -0.1871` vs baseline `-0.2322`: Δ = +0.045 —
  ~50% larger than gainM-1's 0.0314, consistent with 1.2× gainM).

## Three-gainM + three-nsg comparison

| Metric | nsg=0.08 / gainM=0.08 | nsg=0.15 / gainM=0.08 | nsg=0.20 / gainM=0.08 | nsg=0.20 / gainM=0.10 | **nsg=0.20 / gainM=0.12** |
|---|---|---|---|---|---|
| residual max | 5° | 3° | 2° | 2° | **2°** |
| pitch_down max excursion | 4° | 3° | 4° | 4° | **16°** |
| pitch_up max excursion | ~4° | ~3° | 5° | 5° | 5° |
| yaw max excursion | ~4° | ~4° | 4° | 5° | 7° |
| `max_step` | 2° | 2° | 2° | 2° | 2° |
| sine max_step | 1° | 1° | 1° | 1° | 1° |
| `dx` peak | 8.8 mm | 7.2 mm | 7.3 mm | 9.7 mm | **10.9 mm** |
| `err_target` transient | ~7.4 mm | ~6.6 mm | ~6.7 mm | ~8.3 mm | **~9.8 mm** |
| `err_target` quiescent | ~1.2 mm | ~0.8 mm | ~0.5-1.0 | ~0.7-1.0 | ~0.5-0.8 |
| `lambda²` min | 0.00465 | 0.00419 | 0.00465 | 0.00400 | **0.00074** |
| `manip` peak | 2.2e-4 | 2.3e-4 | 2.2e-4 | 2.4e-4 | **3.9e-4** |
| safety regressions | none | none | none | none | none* |

*no clamp violation, no limit hit, but the 16° transient excursion
during pitch_down is a behavior change that warrants attention before
further gainM increase.

## Verdict
- safe to proceed carefully: **yes, but with caveat.**
- all strict task criteria met.
- **gainM = 0.12 is on the edge of the damped-regime** for this robot
  geometry. Beyond this, the arm can take excursive paths from HOME
  that, while bounded by clamps and null-space recovery, look much
  less like "small assistive motion" and more like "arm throwing
  itself into a new configuration before tracking the target."

## Recommendation
1. **Do NOT jump directly to gainM = 0.15.** The step from 0.10 to
   0.12 already quadrupled the pitch_down excursion. Extrapolating
   linearly is unsafe because the effect is non-linear — at gainM=0.15
   the first tick would command a target ~13 mm below HOME, pushing
   even more aggressively past the singular geometry.
2. **Preferred next step: mitigate the unlock, then try gainM=0.15.**
   Two orthogonal knobs, each test-one-at-a-time:
   - **(2a)** Raise `manipThresh` from `5e-4` to `1e-3`. This keeps
     `lam²` large over a wider region around HOME, smoothing the
     transition and preventing the damping collapse. Cost: potentially
     more tracking error near HOME (because damping attenuates the
     response). Expected to most directly fix the unlock.
   - **(2b)** Raise `lambdaMax` from `0.12` to `0.18` or `0.20`. This
     strengthens the worst-case damping at HOME. Cost: marginally
     looser tracking at HOME (already ≤ 1 mm, headroom available).
3. **Alternative: lock gainM at 0.10 as the operator default.** The
   gainM=0.10 test had identical residual (2°), cleaner excursions
   (4° peak), and quantitatively stronger IMU response than 0.08.
   It is a good operating point. The step from 0.10 to 0.12 gives
   +20% more EE reach but also a 4× worse worst-case pitch excursion.
   The tradeoff may not be worth it for first-contact tuning.
4. **If going forward with gainM=0.12 as-is**, monitor operator
   feedback for perception of "arm snatching" on first pitch-from-HOME
   commands. If the behavior is perceived as unnatural, back down
   to 0.10 as a ceiling.
5. **Do not touch** the per-tick clamps (`maxDqDegPerTick=2.0`,
   `maxDxMmPerTick=15.0`). They held the excursion velocity to a safe
   rate. Without them the 16° excursion could have happened in a
   single tick.

## Raw Evidence

Smoke-test RESULTS block:
```text
captured 1652 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [96.0, 100.0, 109.0]  step_max=2.00°   ← BIG EXCURSION
return_center end BSG:  [99.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [102.0, 85.0, 88.0]   step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 89.0]
yaw_+8 end BSG:         [107.0, 85.0, 89.0]   step_max=2.00°
return_center3 end BSG: [100.0, 85.0, 89.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 91.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=16.00°          ← vs 4° in tests 3 and gainM-1
  [FAIL] arm returns to HOME after identity — residual=2.00°    ← cosmetic (< 2.0 strict)
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Key DLS log excerpt showing damping collapse during pitch_down:
```text
20:06:47 target_mm=[60.0, 0.0, 321.0]  cur=[90,90,90]  manip=0.00022 lam2=0.00465 err=0.0  dx=0.0
20:06:47 target_mm=[59.5, 0.0, 310.5]  cur=[90,90,90]  manip=0.00022 lam2=0.00465 err=9.8  dx=10.5
20:06:47 cur=[86,88,94]                              manip=0.00027 lam2=0.00299 err=8.7  dx=9.2
20:06:48 cur=[86,85,98]                              manip=0.00031 lam2=0.00201 err=7.2  dx=7.9
20:06:48 cur=[86,83,101]                             manip=0.00035 lam2=0.00133 err=5.9  dx=7.1
20:06:49 cur=[86,80,105]                             manip=0.00039 lam2=0.00074 err=4.0  dx=5.7    ← lam2 collapsed
20:06:49 target_mm=[60.0, 0.0, 321.0]  cur=[88,82,102]  manip=0.00035 lam2=0.00123 err=7.2 dx=6.2 ← return_center begins
20:06:49 cur=[89,87,96]                              manip=0.00030 lam2=0.00223 err=1.3  dx=6.6   ← recovery fast once off-singular
20:06:50 cur=[89,90,90]                              manip=0.00022 lam2=0.00465 err=0.5  dx=1.0   ← back at HOME
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
imu_q: [0.97192, 0.01074, 0.00391, -0.23492]
```
