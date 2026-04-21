# DLS ASSIST Runtime Config Fix

Targeted runtime-only fix to re-align
`/home/jonny5/raspberry5/config_runtime/robot/routing_config.json`
with the validated tuning baseline. Driven by the task in GitHub
issue `corgiolu-labs/jonny5-slim-workbench#1` (last comment).

## Context
After the final deploy of the validated baseline (commit `8ad990f`)
and the Pi-side service restart (deploy log
`ai/reports/DLS_ASSIST_TUNING_SUMMARY_20260421_204831.md`), the
operator re-opened a stale dashboard session and pressed "Salva
configurazione". The stale page carried the pre-hardening operator
exploration values, which overwrote the persisted
`routing_config.json` with:

- `assistMode = "dls"` (live — the safe default should be `rate`)
- `assistDls = {gainM=0.25, lambdaMax=0.10, manipThresh=7e-4,
  maxDqDegPerTick=3.5, maxDxMmPerTick=28, nullSpaceGain=0.18}`

This is exactly the parameter combination from motion test 5 that
surfaced the damping-unlock / pitch_down 16° excursion. The code
defaults on disk (after commit `8ad990f`) still hold the validated
baseline, but they are only used as fallbacks when the runtime config
omits those keys — with the stale save present, the Pi-side DLS
stack would load those exploratory values on the next mode=5 frame.

## Procedure performed (config-only)
No SSH command executed any motion-inducing operation. No UART
command. No `SETPOSE`. No service restart. No firmware touched. No
STM32 flashed. No Python script run. No smoke test run.

Steps:
1. SSH probe: read current `routing_config.json`; confirm the stale
   state.
2. `shutil.copy2` of the live file to
   `routing_config.json.bak_fix_dls_baseline_20260421_211715`
   (preserves mode/mtime).
3. Atomic rewrite via `tmp + os.replace`:
   - `assistMode` set to `"rate"`.
   - `assistDls` set to the validated baseline (see below).
   - All other top-level keys preserved byte-equivalent.
4. Re-read the file and verify:
   - `assistMode == "rate"` ✓
   - `assistDls == {baseline values}` ✓
   - full key set unchanged ✓

The running `ws_server` (PID 32418, same process as the post-deploy
restart) will pick up the new config via its mtime-based cache at
the next tick in `_load_dx_runtime_cfg`; no restart is needed or
performed.

## Values

### Previous `assistDls` (stale — overwrites from dashboard)
```json
{
  "gainM": 0.25,
  "lambdaMax": 0.10,
  "manipThresh": 0.0007,
  "maxDqDegPerTick": 3.5,
  "maxDxMmPerTick": 28,
  "nullSpaceGain": 0.18
}
```
`assistMode` previous: `"dls"` — the risky one.

### Final `assistDls` (validated baseline)
```json
{
  "gainM": 0.15,
  "lambdaMax": 0.12,
  "manipThresh": 0.001,
  "maxDqDegPerTick": 2.0,
  "maxDxMmPerTick": 15.0,
  "nullSpaceGain": 0.20
}
```
`assistMode` final: **`"rate"`**.

### Backup on Pi
```text
/home/jonny5/raspberry5/config_runtime/robot/routing_config.json.bak_fix_dls_baseline_20260421_211715
```
Created via `shutil.copy2` (preserves permissions and mtime); content
is the full pre-fix state (including the stale `assistDls`) for
deterministic rollback.

## Confirmations
- No motion test was run.
- No UART motion command was issued.
- No firmware touched, no STM32 flashed.
- **No service restart performed** (config-only change, picked up via
  mtime cache in `_load_dx_runtime_cfg`).
- All top-level keys in `routing_config.json` preserved unchanged —
  `armControl`, `controllerMappings`, `headAssist`, `limits`,
  `manualRpyUi`, `pbEn`, `pbState`, `savedAt`, `tuning` are byte-
  equivalent. Only `assistMode` and `assistDls` were rewritten.
- `assistMode` is now `"rate"` — the safe default.

## Remaining risk
- The root cause (stale dashboard page) is not eliminated by this
  fix. If the operator re-opens a stale dashboard and saves again,
  the exploratory values could return. Suggested mitigation (out of
  scope for this fix): reload the dashboard after any source deploy,
  and confirm the sliders show the baseline (gainM=0.15,
  manipThresh=10, etc.) before pressing "Salva configurazione".
- The backup `.bak_fix_dls_baseline_20260421_211715` is on the Pi
  only. The repo does not commit it (runtime config is never
  committed per the task rules).
