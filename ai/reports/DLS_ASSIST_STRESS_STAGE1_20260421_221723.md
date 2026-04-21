# DLS ASSIST Stress Test — Stage 1 (clamp probe, FAIL on anomalous initial state)

Stage 1 of the stress campaign. Intended to isolate the effect of
`maxDxMmPerTick = 15 → 20` with all other parameters at the
validated baseline. Same motion sweep as Stage 0 (±5° pitch, +5°
yaw, ±4° sine).

Prior report: `ai/reports/DLS_ASSIST_DVTEST_20260421_220030.md`.

## Executive Summary
- test executed: **yes** (motion completed, arm recovered to HOME
  via SETPOSE)
- result: **FAIL — data is invalid for the intended experiment**
- **The test is not a valid probe of `maxDxMmPerTick=20` because the
  DLS ran with an anomalous baseline error (`err_target ≈ 27 mm`)
  from the first engaged tick.** The arm then "chased" a target
  shifted by this baseline error throughout the sweep, producing
  distorted excursions, a drifted identity_baseline, and a
  misleading smoke-test verdict.
- **Safety: clean.** `assistMode` restored to `rate`. Arm back at
  mechanical HOME `[100, 88, 93]` via post-test SETPOSE. IMU valid,
  STATUS:IDLE, `max_step ≤ 2°/tick`, no oscillation, no joint
  limit hit.
- **Strict stop-criteria violations** by the stress-plan wording:
  - `err_target` transient **sustained** at `26 – 39 mm` for the
    entire DLS-active window (threshold: `> 18 mm` for `≥ 5` ticks)
  - `dx` cap saturated on **most** ticks of the sweep (threshold:
    `> 10` consecutive saturated ticks)
  - **true** residual from mechanical HOME = **5°** on GOMITO
    (threshold: `> 3°`)
  - (The smoke test's internal verdict reported these as OK because
    it measures against the drifted identity_baseline end state,
    not against mechanical HOME.)

## Parameters (Stage 1)
```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.15,
    "lambdaMax": 0.12,
    "manipThresh": 0.001,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 20.0,
    "nullSpaceGain": 0.20
  }
}
```
Only `maxDxMmPerTick` changed vs Stage 0. Inputs: ±5° pitch, +5°
yaw, ±4° sine (smoke-test standard).

## Environment
- repo commit (workstation): `753934a` (DV-Test report).
- Pi `ws_server` PID **1097** (same instance as Stage 0 rerun and
  DV-Test). Services `jonny5-ws-teleop` + `jonny5-spi-j5vr` active.
- WS client on 8557: `192.168.10.80:64763` (operator dashboard
  MSI.lan).
- Pre-flight BSG = `[100, 88, 93]`, `imu_valid=True`. **YPR not
  verified in pre-flight** — see Root Cause below.
- dirs returned by the firmware mapping (first visible in this
  stage's journal): `[1, -1, 1, 1, -1, 1]` — SPALLA and PITCH are
  physically inverted relative to virt. Confirms prior
  reverse-engineering. HOME physical `[100, 88, 93, 90, 90, 83]` ↔
  virt `[90, 90, 90, 90, 90, 90]`.

## Safety Checks
- Physical routing_config backup (smoke test):
  `routing_config.json.bak_dls_smoke_20260421_221102`.
- Pre-motion smoke-test source backup:
  `verify_dls_assist_smoke.py.bak_stage1_20260421_221054`.
- Script restored from `.bak_stage1_*` after the run; `py_compile`
  clean; inline `test_cfg` back to the committed hardened default.
- `assistMode` final on disk = `rate` (verified).
- Post-test live telemetry: `BSG=[100, 88, 93]`, `YPR=[90, 90, 83]`.
  **Arm physically back at mechanical HOME, including wrist.**
- UART `STATUS:IDLE` throughout.

## The Anomaly

### Symptom
First `[HEAD-ASSIST-DLS]` log line of the run:
```text
22:11:05 target_mm=[60.0, 0.0, 321.0]  cur_virt=[90.0,90.0,90.0]
         next_virt=[90.0, 90.6, 91.6]  manip=0.00016  lam2=0.01011
         err_target=27.8mm  dx=31.1mm
```
- `target = HOME_ee` (identity head quaternion) ✓
- `cur_virt = [90, 90, 90]` (q3 at HOME) ✓
- **BUT `err_target = 27.8 mm`** — impossible if the full 6-DoF
  state is at HOME, since `FK(q=0, q456=0) = HOME_ee` exactly.

Across the whole sweep `err_target` stayed between **26 mm** (at
identity / return_center phases) and **39 mm** (at pitch phase
peaks). It never returned to small values. The DLS committed ≤ 20
mm per tick trying to reduce the error, but the error floor was
structural — driven by a **non-zero `q456` at test start**.

### What this tells us
- `manip = 0.00016` at `cur_virt=[90,90,90]` (Stage 0 and test 7
  had `manip = 0.00022` at the same `cur_virt`). The manipulability
  at q3-HOME depends on q6 as a whole; a shift in q456 changes the
  effective Jacobian and its determinant. The 27 % reduction in
  `manip` is consistent with `q456` being off HOME by several
  degrees in the wrist plane.
- The IMU quaternion at `t=0` of capture was
  `[0.97437, 0.01624, -0.01025, -0.22418]` — similar to prior tests
  (not wildly off), so the wrist was not grossly mis-oriented, but
  the combination of 3 small offsets on Y, P, R was enough to move
  the kinematic EE by ~27 mm relative to HOME_ee.
- The smoke-test capture did not record YPR servo positions (only
  BSG + IMU q). We cannot retroactively confirm the exact wrist
  servo state at the first DLS tick. Post-test YPR = `[90, 90, 83]`
  (HOME), so whatever was off at engagement has since been reset
  by the final SETPOSE.

### What it did to the run
- During `identity_baseline` (1.5 s of head-identity), the DLS
  commanded continuous small joint motion to reduce the baseline
  error. GOMITO **drifted from 93° (HOME) to 97° (+4°) during the
  "identity" hold** — visible in the cap trace:

  ```
  identity_baseline first BSG: [100, 88, 93]  ← HOME
  identity_baseline last  BSG: [100, 88, 97]  ← drifted +4° GOMITO
  ```
- Every subsequent phase started from this drifted state. The
  smoke-test records `home_bsg = baseline_end_BSG = [100, 88, 97]`
  and computes residuals against **that**, not against mechanical
  HOME.
- pitch_down, pitch_up, yaw, sine all proceeded but the commanded
  target shift was added on top of a 27-mm baseline bias. `err`
  moved to 35-39 mm during pitch phases and to 26 mm during
  "identity" phases. `dx` request was 26-39 mm across the whole
  sweep, clamped to 20 mm per tick → sustained cap saturation.
- `final_identity` end BSG = `[98, 89, 98]` → smoke reports
  `residual = 2°` (vs drifted baseline `[100, 88, 97]`). True
  residual vs mechanical HOME `[100, 88, 93]` is `[2, 1, 5] → 5°`.

## Per-phase table (with BOTH smoke-reported and true-HOME excursions)

`home_bsg_smoke` = `[100, 88, 97]` (drifted baseline).
`HOME_mech` = `[100, 88, 93]` (actual mechanical HOME).

| phase | BSG end | max exc from drifted baseline | **max exc from mechanical HOME** |
|---|---|---|---|
| identity_baseline (drift phase) | `[100, 88, 97]` | 0 | **4°** |
| pitch_down_5 | `[ 96, 88, 98]` | 4° | **5°** |
| return_center | `[ 98, 88, 98]` | 2° | 5° |
| pitch_up_neg5 | `[ 98, 89, 97]` | 2° | 5° |
| return_center2 | `[ 98, 88, 98]` | 2° | 5° |
| yaw_right_8 | `[106, 88, 98]` | 6° | **6°** |
| return_center3 | `[ 99, 88, 98]` | 1° | 5° |
| sine_pitch_4deg | `[ 98, 89, 98]` | 2° | 5° |
| final_identity | `[ 98, 89, 98]` | **2°** (smoke verdict) | **5°** |

The smoke-test verdict says `residual=2°` and marks PASS on "arm
returns to HOME after identity". Against mechanical HOME, the
residual is 5° and this is a STOP-criterion violation.

## DLS log — observed ranges (Stage 1)
- `manip`: `0.00016 – 0.00029`
- `lambda²`: `0.00736 – 0.01067` (mitigation plateau held; not the
  cause of the anomaly)
- **`dx` request**: `14 – 39.6 mm`. Cap was 20 mm, so ticks with
  request > 20 mm were clamped. Dominant regime throughout.
- **`err_target`**: `14 – 39 mm` (structural, never below ~14 mm
  even at return_center quiescence)
- `max_step` (physical servo): 2°/tick (clamp held)
- sine `max_step`: 1°/tick (no oscillation)

## Comparison with Stage 0 (same inputs, different maxDx)

| Metric | Stage 0 (maxDx=15) | **Stage 1 (maxDx=20)** | comment |
|---|---|---|---|
| Identity baseline drift | 0 | **4°** (GOMITO) | **Stage 1 anomaly, not explained by the parameter change** |
| `err_target` quiescent | ≤ 1 mm | 26 mm | structural — explained by anomaly |
| `err_target` transient | 12.6 mm | 39 mm | dominated by the baseline offset |
| `dx` peak request | 13.5 mm | 39.6 mm | dominated by the baseline offset |
| True residual from mechanical HOME | 3° | 5° | worse |
| `max_step` | 2°/tick | 2°/tick | = |
| sine max_step | 1°/tick | 1°/tick | = |
| smoke verdict residual | 3° | 2° (misleading) | smoke measures vs drifted baseline |
| Safety | clean | clean | = |

**This comparison is not valid for evaluating `maxDxMmPerTick=20`.**
Stage 1's behavior is dominated by the non-zero initial error, not
by the parameter change.

## Root cause hypotheses

Ranked by plausibility:

1. **`q456` (wrist joints) was not at zero at DLS engagement.**
   Pre-flight verified only BSG, not YPR. Between DV-Test end
   (21:58:38) and Stage 1 start (22:11:02) there's a 12-minute
   gap during which the dashboard client stayed connected. Operator
   interaction (from the dashboard) or firmware HEAD-pipe wrist
   motion triggered by random small orientation blips on the
   operator side could have nudged YPR off HOME. The smoke's
   own SETPOSE at stage start should have reset it, but `wait_sp_done`
   might have timed out or the firmware might have been in a state
   that refused to move YPR the small amount needed — needs direct
   investigation on the firmware side.
2. **Telemetry staleness / cache phase issue.** ws_server has a
   mtime cache on routing_config AND the shared_state telemetry
   cache. If the first mode=5 frame read a slightly old telemetry
   snapshot (say captured before the firmware finished the SETPOSE
   recovery from the DV-Test at the previous run end), the initial
   q6 could differ from the one the smoke read in pre-flight. Less
   likely because the mtime cache re-reads on every change and the
   smoke waits 1.2 s after SETPOSE.
3. **Parameter-change side effect.** Raising `maxDxMmPerTick` from
   15 to 20 could theoretically change the solver's preferred
   initial dq at HOME and push q3 in a direction that destabilizes
   the arm. Extremely unlikely because at HOME with target=HOME
   the exact-arithmetic dq is zero regardless of cap. Ruled out
   unless a numerical FK artifact crosses a threshold at maxDx=20.

Evidence points to **hypothesis 1**: the wrist was off HOME at DLS
engagement and the DLS ran against a 27-mm baseline error for the
entire sweep.

## Verdict — **FAIL**

The test motion completed safely and recovered cleanly, but the
data is invalid for evaluating `maxDxMmPerTick = 20`. Key common
stop criteria (true residual > 3°, err_target sustained > 18 mm,
dx cap saturated > 10 consecutive ticks) were all violated.

## Recommendation

1. **Do NOT proceed to Stage 2.** Stage 1 data is invalid; we cannot
   conclude anything about the effect of the clamp change. Stage 2
   would stack more unknowns.
2. **Re-run Stage 1 with a strengthened pre-flight**: before the
   `verify_dls_assist_smoke.py` invocation, explicitly verify **all
   six** servo positions are at HOME — not just BSG. Suggested:
   ```python
   # Pre-flight snippet (operator adds, outside the motion window):
   python3 -c "import json; d=json.load(open('/dev/shm/j5vr_telemetry.json'));
              ok = all(d[k] == v for k, v in zip(
                 ['servo_deg_B','servo_deg_S','servo_deg_G',
                  'servo_deg_Y','servo_deg_P','servo_deg_R'],
                 [100, 88, 93, 90, 90, 83]));
              print('AT_HOME=', ok)"
   ```
   If `AT_HOME != True`, manually SETPOSE and re-verify before
   running the smoke. Optionally extend the smoke's own pre-HOME
   check (`verify_dls_assist_smoke.py:208-210`) from BSG to the
   full 6-joint set.
3. **If the re-run of Stage 1 also shows the anomaly**, extend the
   investigation:
   - enhance the smoke-test `cap()` to record the full 6-joint
     servo state, not just BSG
   - look at the timing of the SETPOSE HOME → first DLS tick to
     catch any settling transient
   - verify with `journalctl` that `setpose_done` was actually
     received before the smoke proceeded to the motion phase
4. **Safety** is unchanged — arm is physically at HOME, assistMode
   is `rate`, no firmware or STM32 state was altered. The Pi is
   safe to continue using; this is a test methodology issue, not
   a hardware fault.

## Raw Evidence

Smoke-test RESULTS block (misleading due to drifted baseline):
```text
HOME baseline last BSG: [100.0, 88.0, 97.0]   ← NOT HOME — drifted during "identity"
pitch_down_5 end BSG:   [96.0, 88.0, 98.0]    step_max=2.00°
return_center end BSG:  [98.0, 88.0, 98.0]
pitch_up_-5 end BSG:    [98.0, 89.0, 97.0]    step_max=1.00°
return_center2 end BSG: [98.0, 88.0, 98.0]
yaw_+8 end BSG:         [106.0, 88.0, 98.0]   step_max=2.00°
return_center3 end BSG: [99.0, 88.0, 98.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [98.0, 89.0, 98.0]

VERDICT:
  [OK]  pitch_down produced arm motion — delta=4.00°
  [OK]  arm returns to HOME after identity — residual=2.00°   ← MISLEADING
  [OK]  no joint step >5°/frame — max_step=2.00°
```

Anomalous DLS first tick:
```text
22:11:05 [HEAD-ASSIST] assistMode transition: rate -> dls
22:11:05 [HEAD-ASSIST-DLS] target_mm=[60.0, 0.0, 321.0]
                           cur_virt=[90.0,90.0,90.0]
                           next_virt=[90.0, 90.6, 91.6]
                           manip=0.00016  lam2=0.01011
                           err_target=27.8mm   dx=31.1mm   ← anomaly: err ≠ 0 at q=HOME
```

First 5 cap frames (all showing BSG=HOME, IMU steady):
```text
t=0.000 BSG=[100.0, 88.0, 93.0] IMU=[0.9744, 0.0162, -0.0103, -0.2242]
t=0.000 BSG=[100.0, 88.0, 93.0] IMU=[0.9744, 0.0170, -0.0068, -0.2242]
t=0.000 BSG=[100.0, 88.0, 93.0] IMU=[0.9744, 0.0178, -0.0032, -0.2241]
t=0.000 BSG=[100.0, 88.0, 93.0] IMU=[0.9744, 0.0179, -0.0032, -0.2241]
t=0.000 BSG=[100.0, 88.0, 93.0] IMU=[0.9744, 0.0181, -0.0023, -0.2241]
```
BSG was at HOME at capture start, but DLS had already been running
for ~1.5 s warmup and had already drifted GOMITO during that
invisible window. Cap(`t=0`) catches the first frame of the 1.5 s
warm-up's final quiescence before phase 1 begins.

Final routing_config readback:
```text
assistMode = rate
assistDls  = baseline (unchanged)
```

Post-test live telemetry:
```text
BSG  = [100, 88, 93]     ← mechanical HOME restored via SETPOSE
YPR  = [ 90, 90, 83]     ← mechanical HOME restored via SETPOSE
imu_q = [0.96918, 0.01636, 0.01239, -0.24561]
```
