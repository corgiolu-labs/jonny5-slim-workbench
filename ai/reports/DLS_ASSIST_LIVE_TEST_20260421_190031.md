# DLS ASSIST Live Test Report

## Executive Summary
- executed: **no (motion test blocked by deployment-level issues discovered during recon)**
- result: **not-run (safety-stop)**
- reason if not executed:
  SSH recon revealed three pre-existing conditions that make a safe,
  meaningful DLS motion test impossible on the current Pi deployment:
  1. **Telemetry bridge path mismatch.** The SPI bridge writes IMU/servo
     telemetry to `/dev/shm/j5vr_telemetry.json`
     (`controller/teleop/shared_state.py:29`), but the running
     `ws_server.py` reads from `/tmp/j5vr_telemetry.json`
     (`controller/web_services/shared_state.py:28`). The `/tmp` file does
     not exist, so `_extract_servo_physical_deg_from_telemetry()` returns
     `None` every tick. In that state `step_dls_head_assist` short-circuits
     with `hold=True` and no arm motion — the DLS path is effectively inert
     and cannot be observed under motion.
  2. **Pre-existing aggressive DLS config already active.** On connect I
     found `routing_config.json` set to `assistMode=dls` with
     `gainM=0.25`, `lambdaMax=0.10`, `manipThresh=7e-4`,
     `maxDqDegPerTick=3.5`, `maxDxMmPerTick=28`, `nullSpaceGain=0.18` —
     well above the conservative first-contact budget mandated by the
     task. The system did **not** "start in rate" as the task preconditions
     require.
  3. **Pi-deployed code is pre-hardening.** The installed
     `ws_server.py` does not contain the `[HEAD-ASSIST] assistMode
     transition` log line (hardening commit `740c2f3`). Without it the
     task-mandated "confirm log transition `rate -> dls`" evidence cannot
     be captured.

  Additionally, an external WebSocket client was already connected to
  `ws://192.168.10.92:8557` (peer `192.168.10.80:58374`) during recon,
  which means any motion-inducing change I made could have interacted
  with an unknown operator session. All three items require resolution
  before a safe motion test.

  Per the task's SAFETY CONSTRAINTS ("If safety or environment
  availability is uncertain, write a not-run report and stop."),
  I did not run motion.

## Environment
- repo commit before test: `740c2f3` local = `origin/main` pushed
  (`[HARDEN] Improve DLS ASSIST safety and UI config handling`),
  on top of `42482eb` task and `7ae1232` DLS integration.
- Raspberry host used: `Jonny5` — `jonny5@192.168.10.92` via SSH.
- Kernel: `Linux 6.12.47+rpt-rpi-2712 aarch64` (Debian bookworm).
- services checked and state captured:
  - `jonny5-ws-teleop.service` — active (PID 1102, running since
    2026-04-20 06:45 CEST, uptime ~37h).
  - `jonny5-spi-j5vr.service` — active (PID 1101,
    `/home/jonny5/raspberry5/controller/teleop/spi_j5vr_tx.py`).
  - `jonny5-https.service`, `jonny5-https-443-proxy.service`,
    `jonny5-mediamtx.service` — all active (video / dashboard path).
- Deployed repo at `/home/jonny5/raspberry5` is **not a git checkout**
  (no `.git` directory) — it is a deployed snapshot. `git pull` on the
  Pi is not possible; deployment tool required to ship hardening.
- robot hardware physically available: **yes** — IMU streaming valid at
  ~90 Hz, STM32 responding to UART `STATUS?` with `STATUS:IDLE`, arm at
  HOME (B/S/G = 100/88/93 virtual deg, matching the HOME offsets expected
  by `verify_dls_assist_smoke.py:208`).
- IMU telemetry available: **yes** at source
  (`/dev/shm/j5vr_telemetry.json`), but **not reachable by ws_server**
  because of item (1) above.

## Safety Checks
- routing_config physical backup path: `/home/jonny5/raspberry5/config_runtime/robot/routing_config.json.bak_pretest_20260421_185950`
  (3269 bytes, copy of the pre-existing aggressive DLS config, preserved
  `-a` with original permissions).
- original assistMode: **`dls`** (with aggressive parameters — anomaly).
- final assistMode restored to: **`rate`** (set explicitly via atomic
  `tmp_ratepatch` + `os.replace`, per operator instruction
  *"Non lasciare assistMode=dls attivo dopo il test"*). `assistDls`
  sub-block left intact so the operator's tuning values are preserved if
  they want to flip back to `dls` later.
- STOP/E-stop availability: operator present at the workstation with
  SSH session; arm was IDLE throughout, no motion-inducing commands
  issued from my side.
- pre-existing WebSocket client `192.168.10.80:58374` was active on port
  8557. Not identified. All my actions were read-only SPI-side and
  file-level on routing_config; no WS traffic generated from this host.

## Test Parameters
The conservative parameter set specified by the task — to be used when
the deployment blockers are lifted:

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

These values are inside the clamp band of `parse_assist_dls_cfg`
(`head_assist_dls.py:57-78`) and below both the pre-existing Pi config
and the current smoke-script defaults.

## Procedure
Actions actually performed (all read-only except the final `assistMode`
flip and the on-disk backup copy):

1. `git pull` on workstation → already up to date (commits from the
   previous round, including `740c2f3`, already present).
2. `ssh jonny5@192.168.10.92 ...` — confirmed connectivity in BatchMode
   (key-based auth), host `Jonny5`, Debian bookworm ARM64, kernel
   6.12.47+rpt-rpi.
3. Inventory of Pi repo and services:
   - `~/raspberry5` is not a git repo — deployed snapshot.
   - `jonny5-ws-teleop` running `web_services/ws_server.py`,
     `jonny5-spi-j5vr` running `teleop/spi_j5vr_tx.py`.
4. Inspected `routing_config.json`: found `assistMode=dls` with
   aggressive params → flagged as anomaly.
5. Inspected `~/raspberry5/verify_dls_assist_smoke.py`: pre-hardening
   version (no `bak_dls_smoke_*`, no `nullSpaceGain` in inline test_cfg).
   mtime 2026-04-19.
6. Inspected `~/raspberry5/controller/web_services/ws_server.py`: no
   `assistMode transition` log → confirms pre-hardening deployment.
7. Located telemetry bridge paths in two distinct `shared_state.py`
   modules (`teleop/` vs `web_services/`) and confirmed the writer/reader
   path mismatch via file-system inspection:
   - `/dev/shm/j5vr_telemetry.json` exists, 786 bytes, mtime fresh.
   - `/tmp/j5vr_telemetry.json` does **not** exist.
8. Inspected systemd `Environment=` on `jonny5-ws-teleop.service` —
   only `PYTHONPATH` and `WEBRTC_*` set. `J5VR_TELEMETRY_FILE` not set,
   so ws_server falls back to the `/tmp/...` default → mismatch
   confirmed at the environment level.
9. Observed active external WS client `192.168.10.80:58374` on
   `192.168.10.92:8557`. Did not disturb it.
10. Captured IMU baseline from `/dev/shm/j5vr_telemetry.json` directly
    (6 samples at 2 Hz over ~3 s — see Telemetry section below).
11. Created physical backup
    `routing_config.json.bak_pretest_20260421_185950` via `cp -a`.
12. Wrote `assistMode=rate` atomically via a tmp-file + `os.replace`
    Python block on the Pi. Preserved `assistDls` block and all other
    keys. Verified post-write (`assistMode=rate`, `assistDls`
    unchanged, full top-level key set preserved).
13. Confirmed ws-teleop service still in steady state after the config
    change — only `STATUS:IDLE` UART heartbeats in the journal, no
    errors, no motion-related activity.

No WS frames sent from my side. No UART `SAFE`/`ENABLE`/`SETPOSE`
issued. No firmware touched. No STM32 reflash. No service restart.
No deploy-tool invocation.

## Telemetry / IMU Feedback
Captured at baseline (arm static at HOME, no grip activity). Values
read directly from `/dev/shm/j5vr_telemetry.json` (the SPI bridge
sink), bypassing the broken `ws_server` consumer path. 6 samples @ 2 Hz:

| t_rel_s | imu_q_w  | imu_q_x | imu_q_y  | imu_q_z  | B   | S  | G  | Y  | P  | R  | imu_rate_hz | stale |
|---------|----------|---------|----------|----------|-----|----|----|----|----|----|-------------|-------|
| 0.00    | 0.97260  | 0.00989 | -0.00562 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 92.4        | false |
| 0.50    | 0.97260  | 0.00977 | -0.00562 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 90.7        | false |
| 1.00    | 0.97260  | 0.00983 | -0.00549 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 91.6        | false |
| 1.50    | 0.97260  | 0.00989 | -0.00537 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 118.7       | false |
| 2.00    | 0.97260  | 0.00989 | -0.00537 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 93.8        | false |
| 2.50    | 0.97260  | 0.00983 | -0.00525 | -0.23224 | 100 | 88 | 93 | 90 | 90 | 83 | 93.1        | false |

Observations:
- IMU valid, not stale, average rate ~93 Hz (expected ≥90 Hz class).
- Quaternion **static within ~2e-4 on each component** → no drift, no
  noise spike during the 3 s window.
- Yaw component `imu_q_z ≈ -0.232` → ≈ -27° yaw offset in IMU frame
  (static mount bias; consistent across samples).
- BSG = HOME offsets 100/88/93 as expected.
- YPR = 90/90/83 — roll servo sits at 83 (not 90) which is the
  deployed wrist rest pose.
- No DLS log lines captured because (a) no grip-active session ran and
  (b) even if it had, `physical_six=None` would have short-circuited
  before reaching the `[HEAD-ASSIST-DLS]` log site.

Baseline evidence is sufficient to **establish IMU-plane reference** for
a future real test; it is not sufficient to compare commanded direction
vs measured direction, which would require live arm motion.

## Observations
- **Telemetry pipeline mismatch is the primary blocker.** Two
  `shared_state.py` modules ship different default paths for
  `_TELEMETRY_FILE`:
  - `controller/teleop/shared_state.py` → `/dev/shm/j5vr_telemetry.json`
  - `controller/web_services/shared_state.py` → `/tmp/j5vr_telemetry.json`
  Without an aligning `J5VR_TELEMETRY_FILE` env var the producer and
  consumer miss each other, and both mode=5 paths (`rate` and `dls`)
  receive `physical_six=None` and sit in `hold`. The bug is upstream
  of DLS and is not introduced by the DLS commit.
- DLS step code **is** deployed (9 symbol hits in `head_assist_dls.py`
  on the Pi) and matches commit `7ae1232`. It simply never gets live
  data on this deployment.
- `ws_server.py` on the Pi predates the hardening commit `740c2f3` —
  no `assistMode transition` log, no independent assistMode/assistDls
  UI handling. The transitivity of reading config values still works
  because the DLS integration itself was in `7ae1232`.
- Direction correctness / drift / oscillation / lag / saturation:
  **not observable** in this run.
- No sign of STM32 fault — UART returns `STATUS:IDLE` continuously at
  ~0.5 Hz polling (normal).

## Verdict
- safe to continue tuning: **no — environmental blockers must be fixed
  first.** Specifically:
  1. Resolve telemetry file path mismatch. Cheapest option: add
     `Environment=J5VR_TELEMETRY_FILE=/dev/shm/j5vr_telemetry.json`
     to `jonny5-ws-teleop.service` and reload-restart. Longer-term
     fix: align the two `shared_state.py` default paths in the repo.
  2. Deploy the hardening commit `740c2f3` to the Pi. Without it,
     the `[HEAD-ASSIST] assistMode transition` log required by the
     task cannot be emitted.
  3. Update the Pi's `verify_dls_assist_smoke.py` to the hardened
     version, or supersede it with a task-parameter-driven run script.
     The current Pi-deployed copy lacks the physical backup and uses
     slightly looser parameters (`gainM=0.10` vs mandated `0.08`,
     `maxDqDegPerTick=3.0` vs `2.0`, `maxDxMmPerTick=25` vs `15`,
     missing `nullSpaceGain`).
  4. Reset `assistMode` to `rate` as the persistent Pi default — now
     done by this run (was `dls` with aggressive params on arrival).
  5. Identify and coordinate with the external WS client
     (`192.168.10.80`) that was holding an 8557 session, to ensure a
     clear workspace for the next motion test.
- recommended next parameters: exactly the conservative set mandated
  by the task (gainM=0.08, lambdaMax=0.12, manipThresh=5e-4,
  maxDqDegPerTick=2.0, maxDxMmPerTick=15.0, nullSpaceGain=0.08).
- required fixes before next test: items 1-3 above.

## Raw Evidence

Pre-existing routing_config key snapshot (from the Pi):

```text
keys: ['armControl', 'assistDls', 'assistMode', 'controllerMappings',
       'headAssist', 'limits', 'manualRpyUi', 'pbEn', 'pbState',
       'savedAt', 'tuning']
assistMode: dls
assistDls: {'gainM': 0.25, 'lambdaMax': 0.1, 'manipThresh': 0.0007,
            'maxDqDegPerTick': 3.5, 'maxDxMmPerTick': 28,
            'nullSpaceGain': 0.18}
```

Telemetry bridge evidence (path mismatch):

```text
controller/teleop/shared_state.py:29:
  _TELEMETRY_FILE = os.environ.get("J5VR_TELEMETRY_FILE",
                                   "/dev/shm/j5vr_telemetry.json")
controller/web_services/shared_state.py:28:
  _TELEMETRY_FILE = os.environ.get("J5VR_TELEMETRY_FILE",
                                   "/tmp/j5vr_telemetry.json")

On disk:
  /dev/shm/j5vr_telemetry.json  786 bytes  mtime 2026-04-21 19:00:25 (fresh)
  /tmp/j5vr_telemetry.json      MISSING

Service env (systemctl show jonny5-ws-teleop):
  Environment=PYTHONPATH=/home/jonny5/raspberry5
  Environment=WEBRTC_CERT_FILE=...
  Environment=WEBRTC_KEY_FILE=...
  (J5VR_TELEMETRY_FILE not set)
```

SPI bridge liveness (journalctl excerpt):

```text
[SPI-RX TELEMETRY] imu_valid=True raw_byte=1 sample=294534 d_sample=3
  rate=91.6Hz rep=False jump=2 | quat=(w=0.973, x=0.010, y=-0.005, z=-0.232)
  | servo B/S/G/Y/P/R = 100/88/93/90/90/83
```

UART heartbeat (post-config-flip, no anomalies):

```text
19:00:21 [INFO] [UART] cmd=STATUS? seq=1589 response=STATUS:IDLE 13 ms
19:00:23 [INFO] [UART RX MATCH] seq=1590 payload='STATUS:IDLE'
19:00:23 [INFO] [UART] cmd=STATUS? seq=1590 response=STATUS:IDLE 13 ms
19:00:25 [INFO] [UART RX MATCH] seq=1591 payload='STATUS:IDLE'
19:00:25 [INFO] [UART] cmd=STATUS? seq=1591 response=STATUS:IDLE 15 ms
```

Final routing_config state on the Pi:

```text
assistMode= rate
assistDls preserved: {'gainM': 0.25, 'lambdaMax': 0.1,
                      'manipThresh': 0.0007, 'maxDqDegPerTick': 3.5,
                      'maxDxMmPerTick': 28, 'nullSpaceGain': 0.18}
top-level keys unchanged: True

Backup trail on disk:
  routing_config.json.bak-preB1                                  (2026-04-19)
  routing_config.json.bak-preB1-applied-20260419_191042
  routing_config.json.bak-presignpitch-20260419_212004
  routing_config.json.bak_pretest_20260421_185950                (this run)
  routing_config.json                                             (now: assistMode=rate)
```
