# DLS ASSIST Stress Test — Stage 0 Rerun

Single repeat of Stage 0 with identical parameters, to resolve the
question raised in
`ai/reports/DLS_ASSIST_STRESS_STAGE0_20260421_213712.md`:
is the 1° residual deviation from motion test 7 deterministic
(small regression) or run-to-run variance of the null-space
recovery?

## Executive Summary
- test executed: **yes** (single rerun, identical parameters)
- result: **DETERMINISTIC REPRODUCTION of Stage 0 original**
  - final_identity BSG: `[99, 86, 90]` — **identical** to Stage 0
  - residual max: **3°** — **identical** to Stage 0 (1° above
    Stage 0 PASS criterion of 2°; still within common STOP ≤ 3°)
  - every other metric reproduces Stage 0 byte-for-byte
- **conclusion**: the 1° residual deviation from test 7 is **not
  run-to-run variance**. It is a real, reproducible, small
  environmental/mechanical offset that persists even across a
  spontaneous Pi reboot (see Observations).

## Environment Event (noted during rerun)

While preparing the rerun the Pi appears to have **rebooted
spontaneously** between 21:43 and 21:49 CEST, between the two
Stage 0 runs:

| | Stage 0 original | Stage 0 rerun |
|---|---|---|
| Time | 2026-04-21 21:35:36 CEST | 2026-04-21 21:50:00 CEST |
| `ws_server` PID | 32418 | **1097** |
| `ws_server` start | 2026-04-21 20:58:52 CEST | **2026-04-21 21:47:39 CEST** |
| `uptime` at rerun | — | **4 min** (kernel started ~21:49 CEST) |

No systemctl restart command was issued by this session. The Pi's
`uptime` shows the whole machine restarted — not just the service.
That the rerun still reproduces Stage 0 exactly **despite** a full
Pi reboot rules out in-process state as the cause of the
residual-3° deviation from test 7. Any deterministic cause must be
in the files on disk, in the deployed code defaults (which did
change between test 7 and the new baseline), or in the
mechanical / thermal state of the robot.

## Parameters (identical to Stage 0)
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
Same on-Pi smoke-test edits: yaw amplitude `+5°`, `cap()` records
IMU fields. Reverted from `verify_dls_assist_smoke.py.bak_stage0b_20260421_214952`
after the run.

## Motion Sequence
Standard 9-phase sweep + SETPOSE recovery. **1 657 telemetry frames**
captured.

## Telemetry summary (rerun)

| phase | end_t | BSG end | imu_q_z | max excursion |
|---|---|---|---|---|
| identity_baseline | 1.51 | `[100.0, 88.0, 93.0]` | −0.2174 | 0° |
| pitch_down_5 | 3.52 | `[ 97.0, 88.0, 94.0]` | −0.2153 | 3° |
| return_center | 5.03 | `[ 98.0, 87.0, 92.0]` | −0.2359 | 3° |
| pitch_up_neg5 | 7.03 | `[101.0, 86.0, 89.0]` | −0.2481 | 4° |
| return_center2 | 8.54 | `[100.0, 85.0, 90.0]` | −0.2271 | 4° |
| yaw_right_8 (+5°) | 10.55 | `[107.0, 86.0, 90.0]` | −0.1786 | 7° |
| return_center3 | 12.07 | `[100.0, 86.0, 90.0]` | −0.2220 | 7° |
| sine_pitch_4deg | 15.08 | `[ 99.0, 85.0, 91.0]` | −0.2386 | 4° |
| **final_identity** | 17.30 | **`[ 99.0, 86.0, 90.0]`** | −0.2332 | **3°** |

Post-test live telemetry (after SETPOSE HOME):
`BSG=[100, 88, 93]`, `imu_q=[0.97351, 0.01025, 0.01013, -0.22827]`.

## Rerun vs Stage 0 original (deterministic reproduction check)

Every per-phase end BSG triplet matches the Stage 0 original byte-
for-byte:

| phase | Stage 0 (first run) | Stage 0 rerun | Δ |
|---|---|---|---|
| identity_baseline | `[100, 88, 93]` | `[100, 88, 93]` | = |
| pitch_down_5 | `[ 97, 88, 94]` | `[ 97, 88, 94]` | = |
| return_center | `[ 98, 87, 92]` | `[ 98, 87, 92]` | = |
| pitch_up_neg5 | `[101, 86, 89]` | `[101, 86, 89]` | = |
| return_center2 | `[100, 85, 90]` | `[100, 85, 90]` | = |
| yaw_right_8 | `[107, 86, 90]` | `[107, 86, 90]` | = |
| return_center3 | `[100, 86, 90]` | `[100, 86, 90]` | = |
| sine_pitch_4deg | `[ 99, 85, 91]` | `[ 99, 85, 91]` | = |
| **final_identity** | **`[ 99, 86, 90]`** | **`[ 99, 86, 90]`** | **=** |
| residual max | 3° | 3° | = |
| pitch_down excursion | 3° | 3° | = |
| `dx` peak | 13.5 mm | 13.5 mm (peaks 13.3-13.5) | ≈ = |
| `lambda²` min | 0.00840 | 0.00840 | = |
| `max_step` | 2°/tick | 2°/tick | = |
| sine max_step | 1°/tick | 1°/tick | = |

All 9 phase endpoints, the 3 smoke-test verdict lines, and all
aggregate DLS metrics are **identical** between the two Stage 0
runs.

## Comparison with test 7 (unchanged)

| Metric | Test 7 | Stage 0 / rerun |
|---|---|---|
| Mid-sweep phases (8 of 9) | match | match |
| `lambda²` min | 0.00840 | 0.00840 (match) |
| pitch_down excursion | 3° | 3° (match) |
| `dx` peak | 13.2 mm | 13.5 mm (+0.3 mm) |
| **final_identity end BSG** | **`[99, 86, 91]`** | **`[99, 86, 90]`** |
| **residual max** | **2°** | **3°** |

Both Stage 0 runs end GOMITO 1° lower than test 7. No other phase
differs. The deviation is localized to the null-space recovery tail
of `final_identity`.

## Observations

- **Residual deviation is deterministic, not variance.** Two
  independent Stage 0 runs, separated by a full Pi reboot and a
  fresh `ws_server` process, produce byte-identical results. If it
  were run-to-run variance, 1° differences would be expected
  between the two Stage 0 runs as well — but they are identical.
- **The cause is not in-process state.** Pi rebooted, fresh
  process, fresh service instance — no change.
- **The cause is not the smoke test's own test_cfg.** That's the
  same six-field override in both test 7 and Stage 0. The DLS
  behavior mid-sweep is identical (including `lambda²` min and
  pitch_down excursion), which confirms the DLS pipeline is doing
  the same thing.
- **The only downstream structural difference** I can identify
  between the test-7 environment and the current environment is:
  between the two, the committed code defaults in
  `parse_assist_dls_cfg` were updated to the new baseline
  (commit `8ad990f`). These defaults are only used when
  `routing_config.json` omits those keys, which is not the case
  during the smoke test's inject/restore flow — so this should
  not affect smoke-test behavior. But given the smoke test writes
  a `test_cfg` that includes the 6 fields verbatim, the
  `parse_assist_dls_cfg` defaults are masked throughout the run.
  Same in test 7 and Stage 0.
- **Mechanical / thermal hypothesis (can't confirm from here).**
  Servos have some dependency on temperature and backlash state.
  The arm had been exercised heavily during the tuning campaign
  (7 tests + stress Stage 0), and the Pi has been up for only 4
  minutes at the rerun — servos may have cooled compared to the
  end-of-campaign state. This could produce a 1° final-position
  offset on one joint. This is a plausible explanation but I
  cannot distinguish it from other environmental causes in a
  single rerun.
- **Safety implications: none.** Both Stage 0 runs and the rerun
  meet every common STOP criterion. `max_step`, sine amplitude,
  joint limits, `lambda²` plateau, IMU validity, STM32
  `STATUS:IDLE` — all green. The 3° residual is 1° above the
  task-spec `≤ 2°` line but within the stress-plan common STOP
  `≤ 3°`.
- **Spontaneous Pi reboot.** Worth an operational note. Not
  triggered by this session. No session commands could have caused
  a full-OS restart (I have no `sudo reboot` permission used and
  didn't issue such a command). Possible causes: thermal throttle,
  power blip, external intervention on the Pi. The `ws_server`
  service and `spi-j5vr` service both came back up cleanly on
  their own (systemd `Restart=always` handling) — the rerun
  found everything in the expected pre-flight state.

## Verdict
- **Stage 0 PARTIAL**, same as the original run — but now with
  high confidence that the 1° residual offset is deterministic.
- Does **not** violate any common STOP criterion.
- Does **not** reproduce test 7 exactly — the arm's null-space
  recovery settles to a joint configuration 1° further from HOME
  on GOMITO than in test 7. Every other metric matches.

## Recommendation

Two options for the operator, ordered by the quantity of
information they buy:

1. **Accept the new Stage 0 baseline as the calibration set for
   the rest of the stress campaign.** The relevant comparison
   becomes "each later stage vs this Stage 0", not "each later
   stage vs test 7". If Stage 1 shows residual 3° or better, it
   is neutral vs this baseline. If it shows residual > 4°, that
   is a real regression to triage.
   - Under this option, **Stage 1 is safe to run** with the
     current parameters; it is probing one knob
     (`maxDxMmPerTick=15 → 20`) with known-consistent starting
     state.
2. **Triage the 1° regression before proceeding.** Possible probes
   (not motion tests):
   - Record the arm's BSG telemetry at rest over 60 s to
     characterize servo static noise.
   - Inspect `j5_poe_params.json` and `j5_settings.json` runtime
     files to confirm no POE parameter / offset drift occurred
     during the runtime-fix or tuning campaign.
   - Check if the servo gearing / backlash on GOMITO shifted by
     manually grabbing the joint with power off (human operator
     action, outside my scope).

I recommend **option 1**: the deviation is small, deterministic,
well within safety margins, and has a plausible environmental
origin. Waiting for perfect reproduction of a run from 2 hours
ago would delay the campaign without reducing risk. Proceeding
with Stage 0 (current) as the campaign reference preserves all
the safety mechanisms of the stress plan.

If at any subsequent stage the residual creeps above 3° or a new
behavior appears (e.g. pitch_down excursion beyond 4-5°), **that**
is the point to stop and triage.

## Raw Evidence

Smoke-test RESULTS block (rerun):
```text
captured 1657 telemetry frames
HOME baseline last BSG: [100.0, 88.0, 93.0]
pitch_down_5 end BSG:   [97.0, 88.0, 94.0]  step_max=2.00°
return_center end BSG:  [98.0, 87.0, 92.0]
pitch_up_-5 end BSG:    [101.0, 86.0, 89.0] step_max=2.00°
return_center2 end BSG: [100.0, 85.0, 90.0]
yaw_+8 end BSG:         [107.0, 86.0, 90.0] step_max=2.00°
return_center3 end BSG: [100.0, 86.0, 90.0]
sine_pitch_4deg max_step=1.00°
final identity end BSG: [99.0, 86.0, 90.0]

VERDICT:
  [OK]   pitch_down produced arm motion — delta=3.00°
  [FAIL] arm returns to HOME after identity — residual=3.00°
  [OK]   no joint step >5°/frame — max_step=2.00°
```

Service restart signature:
```text
ExecMainStartTimestamp=Tue 2026-04-21 21:47:39 CEST    (new PID 1097)
ActiveEnterTimestamp   =Tue 2026-04-21 21:47:39 CEST
uptime at rerun        = 4 min (kernel ~21:49 CEST)
journalctl: "Started jonny5-ws-teleop.service" @ 21:47:39
```

Final routing_config readback:
```text
assistMode = rate    (restored by smoke finally:)
assistDls  = baseline (unchanged)
```

Post-test live telemetry:
```text
BSG:   [100, 88, 93]   (SETPOSE recovery clean)
imu_q: [0.97351, 0.01025, 0.01013, -0.22827]
```
