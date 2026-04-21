# DLS ASSIST Tuning Campaign — Summary

Closing document for the DLS ASSIST parameter-tuning campaign run on
`jonny5@192.168.10.92` on 2026-04-21. No further motion tests; this
report consolidates the sequence, fixes the code defaults to match
the validated baseline, and records the known-limits / future-work
list.

## Test Sequence (chronological)

| # | Test | gainM | nsg | manipThresh | Key result | Report |
|---|---|---|---|---|---|---|
| 1 | first motion | 0.08 | 0.08 | 5e-4 | residual 5°, partial | `DLS_ASSIST_MOTION_TEST_20260421_191712.md` |
| 2 | null-space sweep | 0.08 | 0.15 | 5e-4 | residual 3°, trend confirmed | `DLS_ASSIST_NULLSPACE_TEST_20260421_192507.md` |
| 3 | null-space sweep | 0.08 | **0.20** | 5e-4 | residual 2°, nsg baseline locked | `DLS_ASSIST_NULLSPACE_TEST3_20260421_194652.md` |
| 4 | gainM sweep | 0.10 | 0.20 | 5e-4 | residual 2°, clean | `DLS_ASSIST_GAINM_TEST1_20260421_200047.md` |
| 5 | gainM sweep | 0.12 | 0.20 | 5e-4 | **16° pitch_down non-linear excursion** | `DLS_ASSIST_GAINM_TEST2_20260421_201058.md` |
| 6 | damping-unlock mitigation | 0.12 | 0.20 | **1e-3** | **excursion 16°→3°**, mitigation validated | `DLS_ASSIST_DAMPING_UNLOCK_TEST1_20260421_202442.md` |
| 7 | gainM sweep | **0.15** | 0.20 | 1e-3 | residual 2°, excursion 3°, clean | `DLS_ASSIST_GAINM_TEST3_20260421_204213.md` |

## Parameters Explored

| Parameter | Touched? | Initial default (pre-campaign) | Final baseline | Values probed |
|---|---|---|---|---|
| `gainM` | yes | 0.22 | **0.15** | 0.08, 0.10, 0.12, 0.15 |
| `lambdaMax` | no | 0.08 | **0.12** (promoted from prior smoke-test value, never varied in sweep) | 0.12 only |
| `manipThresh` | yes | 5e-4 | **1e-3** | 5e-4, 1e-3 |
| `maxDqDegPerTick` | no | 4.0 | **2.0** (promoted from task-mandated clamp, never loosened) | 2.0 only |
| `maxDxMmPerTick` | no | 30.0 | **15.0** (promoted from task-mandated clamp, never loosened) | 15.0 only |
| `nullSpaceGain` | yes | 0.15 | **0.20** | 0.08, 0.15, 0.20 |

(`lambdaMax`, `maxDqDegPerTick`, `maxDxMmPerTick` were never varied
during the sweep — they were fixed at the conservative values mandated
by the task prompt and are promoted to defaults because they held
throughout every test.)

## Main Results

- **Direction correctness** confirmed on all three control axes (pitch
  via Z, yaw via Y, sine via Z) across every test.
- **Null-space drift at HOME** is the limiting factor for residual. It
  shrinks monotonically with `nullSpaceGain`: `5° → 3° → 2°` for
  `nsg = 0.08 → 0.15 → 0.20` at fixed `gainM=0.08`. `nsg=0.20` is the
  smallest value in the probed set that meets the task's `≤ 2°` spec.
- **Commanded EE amplitude scales linearly with gainM** (as expected
  from the `target = HOME_ee + gainM·(d-forward)` formulation).
  Observed: `±7 mm → ±13.1 mm` for `gainM = 0.08 → 0.15`.
- **Per-tick clamps never saturated.** `maxDqDegPerTick = 2.0` was
  touched only on phase-edge transients; `maxDxMmPerTick = 15.0`
  peaked at 13.2 mm (88%) at `gainM=0.15`.
- **EE tracking stays ≤ 1 mm quiescent, ≤ 13 mm transient** across the
  entire sweep. Quiescent is actually best at the higher gainMs (better
  direction alignment in the small-dx regime).
- **Sine oscillation** never exceeded `1°/tick` — null-space bias at
  0.20 does not excite any visible dynamics.
- **Final `assistMode` always restored to `rate`** automatically by
  the smoke test's `finally:` block. No routing_config pollution.

## Problem: damping unlock at `gainM=0.12, manipThresh=5e-4`

Test 5 (gainM=0.12 with the original `manipThresh=5e-4`) surfaced a
non-linear behavior:
- On the **first pitch_down** phase (starting from the kinematically
  singular HOME), the arm drifted to a joint configuration **16° off
  HOME_q** on SPALLA and GOMITO (physical SPALLA 88→100, GOMITO 93→109).
- Root cause: as the arm moves off HOME, Yoshikawa manipulability
  climbs from `2.2e-4` to `3.9e-4` — past 0.78× the `5e-4` threshold.
  The Nakamura/Hanafusa ramp `λ² = (λmax·(1-manip/thresh))²` makes
  `λ²` collapse from `0.00465` to `0.00074` (**6× less damping**).
  The lightly-damped pseudoinverse then commits to its preferred
  antisymmetric (S−, G+) solution for pure-Z motion — a mode that is
  mathematically natural at the rank-deficient HOME Jacobian but
  physically a large joint detour.
- Per-tick clamps kept the trajectory safe (`max_step=2°/tick`,
  arm returned cleanly to HOME at phase end, no limits hit), but the
  intra-phase excursion is 4× what it was at `gainM=0.10`, which
  makes this configuration unsuitable as a default.

## Mitigation: `manipThresh = 1e-3`

Test 6 repeated the `gainM=0.12` run with `manipThresh` doubled to
`1e-3` (all other parameters held). Result:
- Pitch_down max excursion **16° → 3° (−81%)**.
- `λ²` min during pitch_down **0.00074 → 0.00887** (12× more damping
  retained). Baseline `λ²` at HOME also ~doubled (0.00465 → 0.00885).
- SPALLA and GOMITO physical peaks returned to HOME (88° and 93°) —
  the arm barely leaves HOME in joint space during pitch_down.
- `err_target` quiescent effectively unchanged (~0.6-1.0 mm vs
  ~0.5-0.8 mm); transient unchanged.
- `dx` peak effectively unchanged (11.1 mm vs 10.9 mm).
- Residual unchanged (2°).

The Nakamura ramp was simply given a wider operating band:
`thresh=1e-3` keeps `k = 1-manip/thresh` above ~0.6 throughout the
observed manip range, so `λ²` stays firmly in the damped plateau and
the solver remains conservative across the whole HOME neighborhood.

Test 7 then confirmed the mitigation scales: at `gainM=0.15` with
`manipThresh=1e-3`, pitch_down excursion stayed at 3° and every task
criterion remained green.

## Baseline Chosen

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

Committed in this turn as:
- `raspberry/controller/web_services/head_assist_dls.py` —
  `parse_assist_dls_cfg` defaults updated.
- `web/dashboard/js/imu_vr.js` — `_assistDlsDefaults` and the
  `_buildFullPageConfigObject` fallback literals updated.
- `web/dashboard/imu-vr.html` — slider `value=` attributes, tune-value
  spans and the tune-notes updated; added a warning in the "DLS tuning
  avanzato" note that `manipThresh` must stay at or above `10` (1e-3)
  to keep the mitigation active.
- `verify_dls_assist_smoke.py` — residual verdict comparator
  `< 2.0` → `<= 2.0` so the task-spec boundary case (2.00° exactly) no
  longer trips the cosmetic FAIL.

All `parse_assist_dls_cfg` clamp bands are unchanged, so any user
tuning via the dashboard that goes beyond the baseline is still
bounded. `manipThresh` has no upper clamp by design (the Nakamura
ramp handles any value above the observed `manip`), but its lower
bound `1e-6` is unchanged.

## Known Limits

- **HOME is kinematically singular** for 3-DoF position IK. `manip`
  never rises above ~`3.9e-4` in any test even when the arm is 16°
  off HOME_q — the Jacobian geometry in this neighborhood is
  intrinsically rank-deficient in the pitch plane. The tuning
  mitigates this via damping, but it cannot eliminate the underlying
  geometry. Any future change that increases `gainM`, reduces
  `manipThresh`, or reduces `lambdaMax` can re-expose the unlock.
- **`dx` envelope at `gainM=0.15`** is 88% of the 15 mm per-tick cap.
  Going above `gainM=0.15` without first raising `maxDxMmPerTick` will
  probably start clipping on phase-edge transients (the cap would
  bite on the first 1-2 ticks after a target jump, adding visible lag).
- **`assistMode transition` log does not fire between back-to-back
  tests** because the in-process `_last_assist_mode_flag` only updates
  when a mode=5 frame arrives. Between smoke-test runs the dashboard
  is the only client and it does not send mode=5 frames, so the flag
  stays at its prior value. This is by design (the log announces
  transitions, not steady-state), but is worth remembering for audit
  purposes.
- **Pi deployment is not git-managed** (`/home/jonny5/raspberry5` has
  no `.git`). Changes to the committed files on `origin/main` require
  explicit deploy (re-run of the workstation `scp` / deploy-prep flow)
  to take effect on the Pi. The current Pi deployment is up to date
  with commit `740c2f3` (hardening); the default-values patch in
  **this** summary commit must be pushed via the same flow if the
  operator wants the new conservative defaults live on the Pi.
- **`routing_config.json` persistent state on the Pi** still holds
  the older operator exploration (`gainM=0.25`, `lambdaMax=0.10`,
  etc.). Those values are NOT active (`assistMode=rate` is the
  persisted default from this tuning campaign), but they will be
  the values applied if the operator flips `assistMode=dls` from
  the dashboard without saving new tuning first. Suggest the operator
  use the dashboard save flow to persist the new baseline into
  routing_config.
- **Scope** — only the 3-DoF arm subproblem (B/S/G) was tuned. Wrist
  (Y/P/R) is still driven by the firmware HEAD pipe, not DLS. No
  conclusions here apply to the wrist.

## Future Work (ordered by value)

1. **Deploy the updated defaults to the Pi**, then re-run a single
   `verify_dls_assist_smoke.py` with the Pi-deployed defaults (no
   parameter overrides in the script) — confirms the baseline comes
   from the committed code alone.
2. **Operator-side save of the new baseline into `routing_config.json`**
   via the dashboard "Salva configurazione" button, so that a restart
   of `ws_server` (which re-reads routing_config) keeps using the
   tuned values instead of only the code defaults.
3. **Probe `maxDxMmPerTick = 20`** at `gainM=0.15` to check whether
   the DLS wanted to commit faster than 13 mm/tick. If the observed
   `dx` rises noticeably above the old 13.2 mm peak, the current cap
   WAS gating, and a relaxed cap would tighten tracking. If `dx`
   stays ≤ 13 mm, the cap was already non-binding.
4. **Optional: `gainM = 0.18`** combined with `maxDxMmPerTick = 20-25`.
   Predicted `dx` peak ~15.8 mm; would need the raised cap.
5. **Beyond `gainM = 0.18`**: would likely need a combination of
   raised `maxDxMmPerTick`, possibly a higher `maxDqDegPerTick`, and
   potentially a widened `manipThresh` to keep the damping curve
   matched to the larger commanded amplitudes. Not recommended
   without operator in-loop observation.
6. **Alternative residual-reduction probe: `nullSpaceGain = 0.25`**
   with everything else at the baseline. Might bring residual below
   2°, at the cost of slightly stronger return-to-HOME pull that could
   feel resistive to the operator. Low-priority; 2° meets the task
   spec already.
7. **Quaternion-accuracy probe for IMU feedback**: convert captured
   `imu_q_{w,x,y,z}` to RPY and check whether the observed EE pose
   change matches forward-kinematics prediction from the servo BSG
   readback. Useful for validating the mount transform between IMU
   and EE, not for DLS tuning per se.

## Referenced Reports

- `ai/reports/DLS_ASSIST_LIVE_TEST_20260421_185139.md` (not-run,
  workstation only)
- `ai/reports/DLS_ASSIST_LIVE_TEST_20260421_190031.md` (not-run,
  deployment blockers discovered on Pi)
- `ai/reports/DLS_ASSIST_DEPLOY_PREP_20260421_190738.md` (telemetry
  bridge fix + hardening deployed)
- `ai/reports/DLS_ASSIST_MOTION_TEST_20260421_191712.md` (test 1)
- `ai/reports/DLS_ASSIST_NULLSPACE_TEST_20260421_192507.md` (test 2)
- `ai/reports/DLS_ASSIST_NULLSPACE_TEST3_20260421_194652.md` (test 3)
- `ai/reports/DLS_ASSIST_GAINM_TEST1_20260421_200047.md` (test 4)
- `ai/reports/DLS_ASSIST_GAINM_TEST2_20260421_201058.md` (test 5,
  unlock found)
- `ai/reports/DLS_ASSIST_DAMPING_UNLOCK_TEST1_20260421_202442.md`
  (test 6, mitigation validated)
- `ai/reports/DLS_ASSIST_GAINM_TEST3_20260421_204213.md` (test 7,
  final baseline confirmed)

## Summary of Code Changes in This Turn

No motion test was run, no firmware touched, no STM32 flashed, no
runtime config touched on the Pi, no deploy triggered.

Patched in the repo only:
- `raspberry/controller/web_services/head_assist_dls.py`
  (6 default values in `parse_assist_dls_cfg`, plus explanatory comment)
- `web/dashboard/js/imu_vr.js`
  (`_assistDlsDefaults` literal + `_buildFullPageConfigObject` fallbacks)
- `web/dashboard/imu-vr.html`
  (6 slider `value` attributes, 6 tune-value span texts, 2 tune-note
  updates — including the `manipThresh >= 10` safety warning)
- `verify_dls_assist_smoke.py`
  (residual verdict `< 2.0` → `<= 2.0` with explanatory comment)

`py_compile` clean on the three Python files.
