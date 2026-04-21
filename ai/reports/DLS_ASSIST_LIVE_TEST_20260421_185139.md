# DLS ASSIST Live Test Report

## Executive Summary
- executed: **no**
- result: **not-run**
- reason if not executed:
  The task was received and executed from the developer workstation
  (Windows / MINGW64, `C:\Users\pinco\Desktop\TESI\jonny5-vr-robotics-slim-public`),
  **not from the Raspberry Pi**. The controlled live test protocol requires
  operating on the robot host because:
  - `verify_dls_assist_smoke.py` writes to the POSIX runtime-config path
    `/home/jonny5/raspberry5/config_runtime/robot/routing_config.json`,
    which does not exist on the Windows workstation.
  - The smoke test connects to `wss://127.0.0.1:8557` which is the
    loopback WebSocket endpoint of `ws_server.py` running on the Pi.
  - No SSH tunnel to the Pi, no confirmed physical E-stop at hand,
    no live IMU telemetry stream, and no confirmation that the robot
    is safely powered with a clear workspace.

  Per the SAFETY CONSTRAINTS section of
  `ai/tasks/DLS_ASSIST_LIVE_TEST_TASK.md`:
  > If robot/Raspberry/IMU/telemetry is not available or safety cannot
  > be confirmed, do NOT run the live test.

  A not-run report was therefore produced, as instructed.

## Environment
- repo commit before test: `740c2f3` (hardening just pushed on top of `7ae1232` integration and `42482eb` task file).
- Raspberry host used: **none** — executor is the Windows dev workstation.
- services checked from the workstation:
  - local filesystem: `/home/jonny5/...` absent (`uname -s` = `MINGW64_NT-10.0-26200`).
  - Pi WebSocket: not reachable from this shell (no tunnel configured).
- whether robot hardware was physically available: **unknown / not confirmed**.
- whether IMU telemetry was available: **no** — no live `shared_state` telemetry file
  visible from this host.

## Safety Checks
- routing_config physical backup path: **not created** (test not run).
  When the test is eventually run on the Pi, `verify_dls_assist_smoke.py`
  will create `/home/jonny5/raspberry5/config_runtime/robot/routing_config.json.bak_dls_smoke_<TS>`
  before injecting DLS mode (hardening commit `740c2f3`).
- original assistMode: unknown from this host (routing_config lives on the Pi).
  Expected default: `rate` (the backend fallback when the key is absent or
  set to anything other than `"dls"`).
- final assistMode restored to: **not applicable** — nothing was modified.
  Because the runtime config was never touched from this workstation,
  the Pi-side `routing_config.json` is unchanged by this run.
- STOP/E-stop availability noted: **not verified**. A physical test can
  only proceed with operator + E-stop present, which must be confirmed
  at the Pi.

## Test Parameters
The conservative parameter set mandated by the task, to be used
**only** when the test is executed on the Pi:

```json
{
  "assistMode": "dls",
  "assistDls": {
    "gainM": 0.08,
    "lambdaMax": 0.12,
    "manipThresh": 0.0005,
    "maxDqDegPerTick": 2.0,
    "maxDxMmPerTick": 15.0,
    "nullSpaceGain": 0.08
  }
}
```

These values are inside the clamps of `parse_assist_dls_cfg`
(`raspberry/controller/web_services/head_assist_dls.py:57-78`) and
match the "primo test" recommendations from the prior technical review
(gainM 0.08, lambdaMax 0.12, maxDq 2.0-3.0, maxDx 15-25).

Note: the smoke-test script currently hardcodes a slightly less
conservative set (`gainM=0.10`, `maxDq=3.0`, `maxDx=25`, no
`nullSpaceGain` override). Before running, edit those literals in
`verify_dls_assist_smoke.py` inside the `test_cfg["assistDls"] = {...}`
block to match the task's conservative set.

## Procedure
Actions performed in this run (all read-only w.r.t. the robot):

1. `git pull` — fast-forwarded `main` from `7ae1232` to `42482eb`,
   bringing in `ai/tasks/DLS_ASSIST_LIVE_TEST_TASK.md` and
   `ai/AI_COORDINATION.md`.
2. `git status --short` — confirmed the four hardening files were
   still uncommitted in the working tree:
   - `verify_dls_assist_smoke.py`
   - `web/dashboard/imu-vr.html`
   - `web/dashboard/js/imu_vr.js`
   - `raspberry/controller/web_services/ws_server.py`
3. `python -m py_compile raspberry/controller/web_services/head_assist_dls.py
   raspberry/controller/web_services/ws_server.py verify_dls_assist_smoke.py`
   → `py_compile OK`.
4. Staged and committed the four hardening files with the mandated
   message → commit `740c2f3`.
5. `git push origin main` → `42482eb..740c2f3 main -> main`.
6. Environment probe: verified that `/home/jonny5` is absent and
   the platform is `MINGW64_NT-10.0-26200` → confirmed this host
   cannot run the live test.
7. Produced this not-run report under `ai/reports/`.

No UART commands, no `SAFE`/`ENABLE`/`SETPOSE`, no WebSocket connection,
no routing_config mutation, no IMU read.

## Telemetry / IMU Feedback
Not collected. No live test was run.

Expected capture targets (for the eventual run on the Pi):

- baseline HOME/identity:
  - servo_deg_B / _S / _G at offsets (expected ~[100, 88, 93] per
    the smoke-test assertion at `verify_dls_assist_smoke.py:208`).
  - IMU orientation quaternion from backend telemetry.
- pitch_down small input (+5° head pitch): EE should move downward;
  BSG should shift coherently; IMU pitch on the arm should reflect
  the joint motion.
- return_center: EE back to HOME, servo BSG within ~2° of baseline.
- pitch_up (-5° head pitch): mirror of pitch_down.
- yaw small input (+8° head yaw): lateral EE response; check for
  sign consistency with expected direction.
- DLS log lines from `logger.info("[HEAD-ASSIST-DLS] ...")` at
  ~0.5 Hz (every 25 ticks at 50 Hz) — extract `manip`, `lambda_sq`,
  `err_target_mm`, `dx_cart_mm`.

## Observations
None gathered live. Static review observations (carried forward from
the prior technical review):

- DLS math is unchanged; correctness of `np.degrees(q6_math_rad)` in
  `_fk_arm_pos` confirmed against `ik_solver.forward_kinematics_poe`
  contract ("FK from math-space degrees").
- `scipy` is already a declared runtime dependency and already
  transitively imported by `head_assist.py`, so the DLS module does
  not add a new import-failure surface.
- The new `[HEAD-ASSIST] assistMode transition` log (hardening commit
  `740c2f3`) will emit once per mode change, so a clean audit trail
  of `rate -> dls -> rate` will appear in the Pi's service logs.
- The `_validate_routing_config_shape` guard (`runtime_config_paths.py:195`)
  requires `pbState`, `pbEn`, `limits` — the smoke test preserves all
  original keys by round-tripping via `json.loads(json.dumps(original_cfg))`,
  so the injected DLS config still validates.

## Verdict
- safe to continue tuning: **n/a — test not run**.
- recommended next parameters: as documented in the Test Parameters
  section above. Keep `gainM` at 0.08 for first contact; raise to 0.10
  only if IMU feedback shows clean correspondence and no oscillation.
- required fixes before next test: none from this review. Execution
  requirements that must be satisfied on the Pi before attempting:
  1. Operator physically present with E-stop in hand.
  2. Arm in HOME pose with unobstructed workspace (no payload).
  3. `ws_server` service running and IMU telemetry flowing (verify
     `shared_state.read_telemetry_from_file()` populates `servo_deg_*`
     and IMU fields).
  4. Align `test_cfg["assistDls"]` literals in
     `verify_dls_assist_smoke.py` with the conservative set above
     before running.
  5. After the run, verify `routing_config.json` contains `assistMode`
     absent or equal to `"rate"`; keep the `.bak_dls_smoke_*` file on
     disk at least until the operator has visually confirmed correct
     restoration.

## Raw Evidence
Git operations performed from the workstation:

```text
$ git pull
From https://github.com/corgiolu-labs/jonny5-slim-workbench
   7ae1232..42482eb  main       -> origin/main
Fast-forward
 ai/AI_COORDINATION.md                 | 246 +++++++++++++++++++++++++++++++
 ai/tasks/DLS_ASSIST_LIVE_TEST_TASK.md | 263 ++++++++++++++++++++++++++++++++++

$ python -m py_compile raspberry/controller/web_services/head_assist_dls.py \
    raspberry/controller/web_services/ws_server.py verify_dls_assist_smoke.py
py_compile OK

$ git commit -m "[HARDEN] Improve DLS ASSIST safety and UI config handling"
[main 740c2f3] [HARDEN] Improve DLS ASSIST safety and UI config handling
 4 files changed, 41 insertions(+), 9 deletions(-)

$ git push origin main
   42482eb..740c2f3  main -> main
```

Environment probe confirming the workstation is not the Pi:

```text
$ uname -s
MINGW64_NT-10.0-26200

$ ls /home/jonny5
(empty — path does not exist)
```

No robot telemetry, no IMU samples, no DLS log lines to paste —
the test did not run.
