# DLS ASSIST Stress Test Plan

Preparation + pre-flight only. **No motion test, no UART motion
command, no DLS activation, no runtime-config mutation, no service
restart, no code change.** This document defines a staged stress
campaign and the stop criteria — Alessandro must authorize each
stage explicitly before it is executed.

Prior reports consulted:
- `ai/reports/DLS_ASSIST_TUNING_SUMMARY_20260421_204831.md`
- `ai/reports/DLS_ASSIST_GAINM_TEST3_20260421_204213.md`
  (gainM=0.15, current baseline ceiling)
- `ai/reports/DLS_ASSIST_DAMPING_UNLOCK_TEST1_20260421_202442.md`
  (mitigation evidence: why `manipThresh ≥ 1e-3` is load-bearing)
- `ai/reports/DLS_ASSIST_RUNTIME_CONFIG_FIX_20260421_211719.md`
  (baseline restored in routing_config after the stale-dashboard
   incident)

## Pre-flight read-only snapshot (2026-04-21, 21:30 CEST)

### Services
```text
jonny5-ws-teleop.service   active
jonny5-spi-j5vr.service    active
```
`ws_server` PID `32418` (same process as the 20:58 CEST post-deploy
restart); env confirms
`J5VR_TELEMETRY_FILE=/dev/shm/j5vr_telemetry.json`.

### Runtime config (`routing_config.json`)
```json
{
  "assistMode": "rate",
  "assistDls": {
    "gainM": 0.15,
    "lambdaMax": 0.12,
    "manipThresh": 0.001,
    "maxDqDegPerTick": 2,
    "maxDxMmPerTick": 15,
    "nullSpaceGain": 0.2
  }
}
```
Top-level keys present (unchanged): `armControl, assistDls,
assistMode, controllerMappings, headAssist, limits, manualRpyUi,
pbEn, pbState, savedAt, tuning`. Baseline matches the validated
tuning campaign exactly.

### Telemetry (`/dev/shm/j5vr_telemetry.json`)
```text
servo_deg_BSG = [100, 88, 93]            (HOME)
servo_deg_YPR = [ 90, 90, 83]
imu_valid     = True
imu_stale     = False
imu_rate_hz   = 125.4
imu_q         = [0.97363, 0.01105, 0.00629, -0.22778]
mtime age     = 30 ms                    (fresh)
```
STM32 UART heartbeat: `STATUS:IDLE`, ~12 ms round-trip.

### WS clients on :8557
```text
ESTAB 0 91 192.168.10.92:8557 -> 192.168.10.80:59280
```
`192.168.10.80` = `MSI.lan` = Alessandro's operator workstation
(confirmed in all prior sessions via reverse DNS and MAC OUI).
**No unknown WS clients observed.**

### Pre-flight verdict
All eight gates are green:

| Gate | Status |
|---|---|
| ws-teleop service active | ✓ |
| spi-j5vr service active | ✓ |
| `assistMode == "rate"` | ✓ |
| `assistDls` matches validated baseline | ✓ |
| IMU valid, not stale, ≥ 90 Hz | ✓ (125 Hz) |
| Servo B/S/G at HOME | ✓ ([100, 88, 93]) |
| STM32 firmware idle | ✓ (STATUS:IDLE) |
| No unknown WS clients on 8557 | ✓ (only operator dashboard) |

## Campaign Overview

Four staged tests, executed one at a time with operator authorization
between stages. Each stage uses the same motion sweep
(`verify_dls_assist_smoke.py`, identity → pitch_down_5 → ...
→ final_identity → SETPOSE HOME), with yaw capped at +5°.

`nullSpaceGain=0.20`, `manipThresh=0.001`, `lambdaMax=0.12`,
`maxDqDegPerTick=2.0` are held constant through all stages.
The only parameters that vary are `gainM` and `maxDxMmPerTick`.

| Stage | `gainM` | `maxDxMmPerTick` | Priority | Risk |
|---|---|---|---|---|
| 0 — baseline confirmation | 0.15 | 15 | **MUST** (confirms nothing regressed) | very low |
| 1 — clamp probe | 0.15 | 20 | high | low |
| 2 — gain moderate | 0.18 | 20 | medium | moderate |
| 3 — upper probe (optional) | 0.20 | 20 | optional | moderate-high |

No stage above `gainM=0.20` or `maxDxMmPerTick=20` is proposed.
`maxDqDegPerTick` stays at 2.0 throughout (per task rules).

### Stage 0 — baseline confirmation
- **Objective:** confirm that after the runtime-config fix (commit
  `19f34ff`) the Pi still produces the same behavior the campaign
  closed on. This is a "smoke test of the smoke test" — if stage 0
  deviates from the recorded stage-7 results, something changed
  beyond what we know.
- **Parameters**
  ```json
  {"assistMode":"dls","assistDls":{"gainM":0.15,"lambdaMax":0.12,"manipThresh":0.001,"maxDqDegPerTick":2.0,"maxDxMmPerTick":15.0,"nullSpaceGain":0.20}}
  ```
- **Metrics to collect** (from both smoke stdout and journalctl):
  - servo BSG per phase + max excursion from HOME
  - IMU `imu_q_{w,x,y,z}` per phase
  - DLS log: `manip`, `lambda²`, `err_target_mm`, `dx_cart_mm`
  - smoke-test verdict (now with the fixed `≤ 2.0` comparator)
  - `[HEAD-ASSIST] assistMode transition: rate -> dls` line
- **PASS criteria**
  - pitch_down max excursion ≤ 5° (expected 3°, allow +2° margin)
  - residual ≤ 2° (task spec)
  - `err_target` quiescent ≤ 3 mm
  - `dx` peak ≤ 15 mm (cap not exceeded)
  - `max_step` ≤ 2°/tick, sine `max_step` ≤ 1°/tick
  - `lambda²` min ≥ 0.005 during pitch_down (mitigation intact)
  - final `assistMode == "rate"` on disk after smoke `finally:`
- **STOP criteria**
  - any of the common stop criteria below
  - pitch_down max excursion > 6°: mitigation may have drifted;
    re-verify `manipThresh=1e-3` on disk and compare to prior runs
- **Risk:** very low — identical to test 7 which passed cleanly.
- **Preparation needed (DO NOT EXECUTE YET)**:
  - `cp -a verify_dls_assist_smoke.py verify_dls_assist_smoke.py.bak_stress_0`
  - patch `test_cfg["assistDls"]` to stage-0 values + add IMU capture
    + cap yaw at +5° (same pattern as tests 1-7)
  - `python3 -m py_compile verify_dls_assist_smoke.py`
  - run `verify_dls_assist_smoke.py` via `.venv/bin/python3`, capture
    `journalctl -u jonny5-ws-teleop --since "<start>"`

### Stage 1 — clamp probe
- **Objective:** determine whether `maxDxMmPerTick=15` was gating at
  `gainM=0.15` (test 7 peaked at 13.2 mm, 88% of cap). If the DLS
  wanted to commit faster, raising the cap will reduce `err_target`
  transient. If `dx` peak stays near 13 mm even with the cap raised
  to 20, the cap was non-binding and the result informs whether to
  promote cap=20 to default.
- **Parameters**
  ```json
  {"assistMode":"dls","assistDls":{"gainM":0.15,"lambdaMax":0.12,"manipThresh":0.001,"maxDqDegPerTick":2.0,"maxDxMmPerTick":20.0,"nullSpaceGain":0.20}}
  ```
- **Metrics to collect** (same as stage 0, plus):
  - `dx` peak specifically — the primary question
  - `err_target` transient max — secondary outcome
- **Predictions**
  - `dx` peak: if cap was gating, expect 13.5-16 mm; if non-binding,
    ≈ 13 mm unchanged
  - `err_target` transient: if cap was gating, expect lower than test
    7's 12.4 mm; if non-binding, ≈ 12 mm unchanged
  - residual: unchanged at 2°
  - pitch_down excursion: unchanged at ≤ 5°
- **PASS criteria**
  - pitch_down max excursion ≤ 7°
  - residual ≤ 2°
  - `err_target` quiescent ≤ 3 mm, transient ≤ 18 mm (the raised-cap
    budget)
  - `dx` peak ≤ 20 mm (cap not exceeded)
  - `max_step` ≤ 2°/tick, sine `max_step` ≤ 1°/tick
- **STOP criteria**: common list below.
- **Risk:** low — we are relaxing one clamp by +33% while keeping all
  others at conservative. `maxDqDegPerTick=2.0` still limits the
  per-joint rate independently, so the arm speed envelope is
  constrained by the dq clamp even if dx is relaxed.

### Stage 2 — gain moderate
- **Objective:** explore the smallest gain step beyond the validated
  ceiling (`gainM=0.15 → 0.18`, +20%). Requires the relaxed clamp
  from stage 1 because at `gainM=0.18` the predicted `dx` peak is
  ≈ 15.8 mm (above the 15 mm cap), so stage 1 must have passed
  before stage 2 is attempted.
- **Parameters**
  ```json
  {"assistMode":"dls","assistDls":{"gainM":0.18,"lambdaMax":0.12,"manipThresh":0.001,"maxDqDegPerTick":2.0,"maxDxMmPerTick":20.0,"nullSpaceGain":0.20}}
  ```
- **Metrics to collect** (same as stages 0-1, with special attention to):
  - pitch_down excursion — the non-linear-runaway risk indicator
  - `lambda²` min during pitch_down — whether the mitigation still
    holds at this higher gain
- **Predictions** (linear extrapolation from test 7):
  - target EE amplitude ≈ ±15.7 mm (×1.20 vs stage 1's 13.1 mm)
  - `dx` peak ≈ 15-16 mm (under 20 mm cap)
  - `err_target` transient ≈ 14 mm
  - residual: still 2° (null-space at 0.20 should still handle)
  - pitch_down excursion: still ≤ 5° **if** the damping mitigation
    holds; if damping collapses, could jump back toward 16° — that's
    precisely the STOP trigger
- **PASS criteria**
  - pitch_down max excursion ≤ 7° (stricter than general stop — we
    want to see the mitigation hold cleanly, not just "not runaway")
  - residual ≤ 2°
  - `err_target` quiescent ≤ 3 mm, transient ≤ 18 mm
  - `dx` peak ≤ 20 mm
  - `lambda²` min during pitch_down ≥ 0.005 (mitigation envelope
    intact)
  - `max_step` ≤ 2°/tick, sine `max_step` ≤ 1°/tick
- **STOP criteria**: common list PLUS: pitch_down excursion > 8° →
  revert to stage-1 config.
- **Risk:** moderate. This is the first test beyond the validated
  ceiling. The damping-unlock mechanism is known (test 5) and the
  mitigation is known effective at 0.12 and 0.15. Jumping to 0.18
  is a +20% step, not a +50% step, so extrapolation is reasonable.

### Stage 3 — upper probe (OPTIONAL)
- **Objective:** find where the practical usable gain maxes out.
  Not required for a usable operating point (0.15 already works;
  0.18 if stage 2 passes will be a sufficient upper default). Run
  this stage only if the operator wants to know where the envelope
  ends.
- **Parameters**
  ```json
  {"assistMode":"dls","assistDls":{"gainM":0.20,"lambdaMax":0.12,"manipThresh":0.001,"maxDqDegPerTick":2.0,"maxDxMmPerTick":20.0,"nullSpaceGain":0.20}}
  ```
- **Metrics to collect**: same as stages 0-2.
- **Predictions**
  - target EE amplitude ≈ ±17.5 mm
  - `dx` peak ≈ 17 mm (approaching 20 mm cap)
  - `err_target` transient ≈ 16 mm
  - residual: 2° (if mitigation holds)
  - pitch_down excursion: uncertain — we are near the upper edge
    of the tested regime; possible that clamps start biting and the
    arm can't catch the amplitude
- **PASS criteria**
  - pitch_down max excursion ≤ 10° (wider than stages 0-2 because
    this is the edge)
  - residual ≤ 3° (slightly relaxed — we're not promoting this to
    default)
  - `err_target` quiescent ≤ 3 mm, transient ≤ 18 mm
  - `dx` peak ≤ 20 mm
  - no `dx` clamp saturation on ≥ 5 consecutive ticks (a brief touch
    is OK; sustained is a lag problem)
  - `max_step` ≤ 2°/tick, sine `max_step` ≤ 1°/tick
- **STOP criteria**: common list PLUS: pitch_down excursion > 10° OR
  sustained `dx` saturation → back off to stage 2 as the ceiling.
- **Risk:** moderate-high. Possible outcomes: (a) clean pass → 0.20
  is usable; (b) clamps bite and tracking lags → 0.18 is the
  ceiling; (c) damping unlocks despite manipThresh=1e-3 → 0.20 is
  out of envelope for this hardware and `manipThresh` would need
  widening (e.g. 1.5e-3). Option (c) is unlikely given stage 2
  expectations, but possible.

## Common STOP criteria (apply to every stage)

Immediately abort, restore `assistMode=rate`, and report if any of
the following are observed during or immediately after the motion
sweep:

- **Direction error on any axis** (e.g. pitch_down moves arm up)
- **Sudden / unexpected motion** at start of any phase
- **pitch_down max excursion > 10°** (stage 0-2: stricter stage
  criteria override this)
- **final residual > 3°**
- **`err_target` final quiescent > 3 mm**
- **`err_target` transient sustained > 18 mm** for ≥ 5 consecutive
  DLS ticks
- **`dx` clamp saturation** (`dx` == `maxDxMmPerTick`) on **≥ 5
  consecutive ticks** — indicates the cap is gating throughout the
  phase, not just at transitions
- **`max_step > 2°/tick`** on any tick (should be impossible unless
  the clamp in code is broken; would flag a regression in the step
  clamp)
- **sine phase `max_step > 2°/tick`** OR any oscillation visible
  in the BSG trace across ticks
- **IMU invalid, stale, or rate < 90 Hz** at any point
- **STM32 UART `STATUS:FAULT`** or any `STATUS != IDLE` post-motion
- **`assistMode != "rate"`** after the smoke test's `finally:`
  block completes

## Metrics collection (standardised across stages)

For every stage, record:

### Per-phase (9 phases, plus post-SETPOSE-recovery):
- servo BSG at end of phase
- max |BSG − HOME| during phase (joint excursion)
- IMU `imu_q_{w,x,y,z}` at end of phase

### Aggregate across the motion window:
- `manip` min, max
- `lambda²` min, max (with special attention to pitch_down minimum)
- `err_target_mm` quiescent steady, transient max
- `dx_cart_mm` peak per tick
- smoke-test verdict (`pitch_down produced arm motion`,
  `arm returns to HOME after identity`, `no joint step >5°/frame`)
- journalctl excerpts:
  - `[HEAD-ASSIST] assistMode transition: ... -> dls`
  - first 3 and last 3 `[HEAD-ASSIST-DLS]` lines of each DLS-active
    interval
  - any `[HEAD-ASSIST] grace hold` lines
  - any WARNING / ERROR / TRACEBACK lines

### Environment fingerprint at start of stage:
- ws_server PID + `/proc/<pid>/environ` hash (to detect accidental
  restart or config drift)
- `routing_config.json` sha256 (to detect mid-run tampering)
- IMU baseline quaternion at entry (for direction-correctness)

### Final state:
- `assistMode` on disk after smoke `finally:` (must be `rate`)
- BSG on disk after post-test SETPOSE (must be `[100, 88, 93]`)
- STM32 `STATUS:IDLE` within 30 s of motion end

## Template — per-stage report

Future stress-test reports should use this skeleton (mirrors the
existing `DLS_ASSIST_GAINM_TEST*` format so comparisons are 1:1):

```markdown
# DLS ASSIST Stress Test — Stage N

## Executive Summary
- stage: N (name)
- parameters:
- result: PASS | PARTIAL | FAIL | ABORTED
- headline metric (pitch_down excursion, dx peak, etc.)
- safety notes

## Environment
- commit (workstation):
- ws_server PID:
- WS clients on 8557:

## Safety Checks
- routing_config backup path:
- smoke-test source backup path:
- joint limits vs peaks:
- assistMode final:
- post-test SETPOSE BSG:

## Motion Sequence
- frames captured:
- per-phase table: (end_t, BSG, imu_q, max_excursion)

## DLS log ranges
- manip, lambda², dx, err_target, max_step, sine max_step

## Comparison with previous stages / baseline
- table vs stage N-1 / vs test 7

## Observations
- damping profile, clamps, null-space behavior, direction correctness

## Verdict + Recommendation
- proceed to stage N+1 / lock at stage N / revert

## Raw Evidence
- smoke-test stdout
- journalctl excerpts
- final routing_config readback
- post-test telemetry readback
```

## First stage to run

**Start with Stage 0 (baseline confirmation).**

Rationale:
- Zero new risk (identical parameters to the validated test 7).
- Confirms nothing regressed between commit `09172d0` (test 7) and
  the current deploy `8ad990f` + runtime-fix `19f34ff`.
- If stage 0 does not reproduce test 7 within tolerance, do **not**
  proceed to stages 1-3; first triage the regression.
- Stage 0 is also the cleanest calibration dataset — any subsequent
  stage interprets its results relative to stage 0, not relative to
  the historical test 7 (which ran on a different ws_server PID and
  before the runtime-fix incident).

Only after stage 0 passes should Alessandro authorize stage 1.
Each subsequent stage requires explicit operator authorization.

Actions **not performed** in this document:
- No smoke test executed.
- No motion command issued.
- No UART motion command issued.
- No firmware touched, no STM32 flashed.
- No routing_config mutation.
- No service restart.
- No code modification.
- No script patch (the smoke-test script on the Pi is still the
  committed hardened version with the `gainM=0.10` smoke preset —
  it will need the usual on-Pi patch to the staged parameters
  before each stage, and that patch must be prepared at the moment
  of authorization, not now).
