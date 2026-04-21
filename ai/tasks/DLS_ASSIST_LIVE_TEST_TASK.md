# TASK — Controlled DLS ASSIST Live Test with IMU Feedback

Target executor: Claude Code

Repository local path expected on Alessandro's PC:

```text
C:\Users\pinco\Desktop\TESI\jonny5-vr-robotics-slim-public
```

---

## OBJECTIVE

Prepare and, only if the local Raspberry/JONNY5 environment is available and safe, run a controlled first live test of the DLS ASSIST mode using IMU telemetry as feedback.

Commit and push the test report/results so ChatGPT can review them from GitHub.

---

## IMPORTANT CONTEXT

- DLS ASSIST integration commit: `7ae1232`.
- A local hardening patch was already applied by Claude Code:
  - physical backup in `verify_dls_assist_smoke.py`
  - clearer DLS gain UI label
  - independent reading of `assistMode` / `assistDls`
  - non-spam log for ASSIST mode transitions
- That hardening patch may still be uncommitted locally. First check `git status`.
- The DLS math must NOT be changed.
- The historical `rate` path must remain the default fallback.

---

## SAFETY CONSTRAINTS

Do NOT:

- Modify firmware.
- Flash STM32.
- Change DLS math.
- Refactor.
- Touch unrelated files.
- Force-push.
- Delete backups/logs/config files.
- Leave `assistMode=dls` active after the test.
- Commit runtime config containing `assistMode=dls`.
- Commit large logs.
- Commit secrets, credentials, private keys, or environment files.

Runtime config may be modified only through the controlled smoke test flow with physical backup and final restore.

If robot/Raspberry/IMU/telemetry is not available or safety cannot be confirmed, do NOT run the live test. Instead prepare the test plan and commit the report saying why the test was not executed.

---

## FIRST STEP — COMMIT/PUSH HARDENING PATCH IF PRESENT

1. Run:

```bash
git status --short
```

2. If these files are modified:

```text
verify_dls_assist_smoke.py
web/dashboard/imu-vr.html
web/dashboard/js/imu_vr.js
raspberry/controller/web_services/ws_server.py
```

then run:

```bash
python -m py_compile raspberry/controller/web_services/head_assist_dls.py raspberry/controller/web_services/ws_server.py verify_dls_assist_smoke.py
```

3. If `py_compile` passes, commit and push only those hardening files with message:

```text
[HARDEN] Improve DLS ASSIST safety and UI config handling

- Add physical routing_config backup in DLS smoke test
- Clarify DLS gain UI semantics
- Read assistMode and assistDls independently from headAssist
- Log ASSIST mode transitions without spam
```

---

## SECOND STEP — CONTROLLED LIVE TEST PREPARATION

Use conservative DLS parameters only:

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

Live test goals:

- Verify no drift at identity head quaternion.
- Verify small head pitch up/down produces expected arm vertical behavior.
- Verify small yaw left/right produces expected lateral behavior.
- Use IMU feedback to compare expected direction vs measured IMU response.
- Capture DLS log lines: `[HEAD-ASSIST-DLS] ... manip=... err_target=... dx=...`.
- Capture relevant telemetry: `servo_deg_B/S/G` and IMU orientation/pitch/roll/yaw if available.
- Confirm final restore to historical/default `assistMode=rate` or original `routing_config`.

---

## IMU FEEDBACK REQUIREMENT

Use available backend telemetry/logs to collect IMU orientation feedback during the test.

The report must include:

- baseline IMU orientation at HOME/identity
- IMU response for pitch_down small input
- IMU response for pitch_up small input
- IMU response for yaw small input if available
- whether measured IMU direction agrees with intended direction
- any lag, drift, oscillation, or sign inversion observed

If there is an existing IMU validation/self-test pipeline in the repo, prefer using it read-only or as an observation tool. Do not modify it unless necessary and explicitly scoped.

---

## LIVE TEST EXECUTION RULES

- Before enabling DLS, verify system starts in `rate`.
- Confirm log transition `rate -> dls`.
- Run only small motions.
- Keep E-stop/STOP ready.
- If drift, oscillation, wrong direction, or sudden movement appears: STOP test and restore config.
- Always restore original routing_config at the end.

---

## REPORT REQUIREMENT

Create a Markdown report under:

```text
ai/reports/DLS_ASSIST_LIVE_TEST_<YYYYMMDD_HHMMSS>.md
```

Report format:

```markdown
# DLS ASSIST Live Test Report

## Executive Summary
- executed: yes/no
- result: pass/fail/partial/not-run
- reason if not executed

## Environment
- repo commit before test
- Raspberry host used
- services checked
- whether robot hardware was physically available
- whether IMU telemetry was available

## Safety Checks
- routing_config physical backup path
- original assistMode
- final assistMode restored to
- STOP/E-stop availability noted

## Test Parameters
Include exact JSON used.

## Procedure
Step-by-step actual commands/actions performed.

## Telemetry / IMU Feedback
Include compact tables or bullet logs for:
- baseline HOME/identity
- pitch_down
- return_center
- pitch_up
- yaw if tested

Include:
- servo B/S/G trends
- IMU pitch/roll/yaw trends if available
- DLS manip/err_target/dx log samples

## Observations
- direction correctness
- drift
- oscillation
- lag
- saturation
- unexpected behavior

## Verdict
- safe to continue tuning: yes/no
- recommended next parameters
- required fixes before next test

## Raw Evidence
Paste relevant log excerpts, not huge full logs.
```

---

## COMMIT/PUSH REQUIREMENT

After the test or not-run report:

1. Run:

```bash
git status --short
```

2. Commit only:

- the hardening files if not already committed
- the new report file under `ai/reports/`

3. Push to `origin main`.

4. Final output must include:

- commit hash(es)
- report file path
- whether live test executed
- final assistMode confirmed/restored
- any red flags

---

## DO NOT COMMIT

- Runtime config containing `assistMode=dls`
- Large logs
- Secrets
- IP credentials
- Private keys
- `.env` files
- unrelated generated files

---

## FINAL NOTE

The goal is a safe first-contact test, not performance tuning.

If safety or environment availability is uncertain, write a not-run report and stop.
