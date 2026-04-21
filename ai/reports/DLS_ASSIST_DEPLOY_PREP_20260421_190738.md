# DLS ASSIST Deploy Prep Report

Target Pi: `jonny5@192.168.10.92` (`Jonny5`, Debian bookworm ARM64,
kernel `6.12.47+rpt-rpi-2712`).

Based on findings of `ai/reports/DLS_ASSIST_LIVE_TEST_20260421_190031.md`.

## What was modified

All changes are minimal and reversible. Backups listed below.

1. **systemd unit** `/etc/systemd/system/jonny5-ws-teleop.service`
   - Added one line to the `[Service]` section:
     ```
     Environment=J5VR_TELEMETRY_FILE=/dev/shm/j5vr_telemetry.json
     ```
   - Inserted immediately after the existing `Environment=WEBRTC_KEY_FILE=...`
     via `sudo sed -i "/^Environment=WEBRTC_KEY_FILE=/a ..."`.
   - Aligns the `ws_server.py` consumer path with the SPI-bridge writer
     path (`controller/teleop/shared_state.py` default), without touching
     either `shared_state.py` module.

2. **Hardening file deploy** (copy-in-place from workstation to Pi
   deploy snapshot `/home/jonny5/raspberry5`, which is not a git
   checkout). Files copied verbatim from commit `740c2f3`:
   | Source on workstation | Destination on Pi |
   |---|---|
   | `raspberry/controller/web_services/ws_server.py` | `~/raspberry5/controller/web_services/ws_server.py` |
   | `verify_dls_assist_smoke.py` | `~/raspberry5/verify_dls_assist_smoke.py` |
   | `web/dashboard/imu-vr.html` | `~/raspberry5/web/dashboard/imu-vr.html` |
   | `web/dashboard/js/imu_vr.js` | `~/raspberry5/web/dashboard/js/imu_vr.js` |

   Verified after deploy:
   - `ws_server.py:215` contains `logger.info("[HEAD-ASSIST] assistMode transition: %s -> %s", ...)`
   - `verify_dls_assist_smoke.py` contains `bak_dls_smoke_{ts}` and `Physical backup written:`
   - `imu-vr.html:246` uses new label `Reattivita' braccio (m)` with explanatory note
   - `imu_vr.js` lines 551/573/584: independent `_touchedHeadAssistUi` flow for `assistMode` / `assistDls`
   - `python3 -m py_compile` on `ws_server.py`, `head_assist_dls.py`, `verify_dls_assist_smoke.py` → OK

3. **routing_config**: already flipped to `assistMode=rate` in the
   previous session (see `DLS_ASSIST_LIVE_TEST_20260421_190031.md`).
   Confirmed still `rate` at the start and end of this session.
   `assistDls` sub-block (`gainM=0.25`, `lambdaMax=0.1`,
   `manipThresh=7e-4`, `maxDqDegPerTick=3.5`, `maxDxMmPerTick=28`,
   `nullSpaceGain=0.18`) preserved untouched as the operator's stored
   aggressive tuning — recorded here for reference; not used.
   No firmware changed, no STM32 flash, no UART motion command issued.

## Backups created (all on the Pi)

| File | Backup path | Pre-deploy bytes |
|---|---|---|
| `ws_server.py` | `~/raspberry5/controller/web_services/ws_server.py.bak_predeploy_20260421_190625` | 20 550 |
| `verify_dls_assist_smoke.py` | `~/raspberry5/verify_dls_assist_smoke.py.bak_predeploy_20260421_190625` | 12 565 |
| `imu-vr.html` | `~/raspberry5/web/dashboard/imu-vr.html.bak_predeploy_20260421_190625` | 19 580 |
| `imu_vr.js` | `~/raspberry5/web/dashboard/js/imu_vr.js.bak_predeploy_20260421_190625` | 38 156 |
| systemd unit | `/etc/systemd/system/jonny5-ws-teleop.service.bak_predeploy_20260421_190625` | 552 |

Previous routing_config physical backup from the prior session also
still on disk: `routing_config.json.bak_pretest_20260421_185950` (3 269 bytes).

Rollback (if ever needed) is:
```bash
sudo cp -a /etc/systemd/system/jonny5-ws-teleop.service.bak_predeploy_20260421_190625 \
          /etc/systemd/system/jonny5-ws-teleop.service
cp -a ~/raspberry5/controller/web_services/ws_server.py.bak_predeploy_20260421_190625 \
      ~/raspberry5/controller/web_services/ws_server.py
cp -a ~/raspberry5/verify_dls_assist_smoke.py.bak_predeploy_20260421_190625 \
      ~/raspberry5/verify_dls_assist_smoke.py
cp -a ~/raspberry5/web/dashboard/imu-vr.html.bak_predeploy_20260421_190625 \
      ~/raspberry5/web/dashboard/imu-vr.html
cp -a ~/raspberry5/web/dashboard/js/imu_vr.js.bak_predeploy_20260421_190625 \
      ~/raspberry5/web/dashboard/js/imu_vr.js
sudo systemctl daemon-reload && sudo systemctl restart jonny5-ws-teleop.service
```

## Services restarted

- `jonny5-ws-teleop.service` — one restart after `daemon-reload`.
  - Old PID 1102 (running since 2026-04-20 06:45 CEST, ~37h uptime).
  - New PID **12572**, started 2026-04-21 19:06:56 CEST.
  - Startup sequence in journal clean: `SET_JOINT_LIMITS` for all 6
    joints, `SET_OFFSETS 100 88 93 90 90 83`, `IMUON`, `SET_VR_PARAMS`,
    `STATUS:IDLE` from STM32.
  - No `error`/`exception`/`traceback`/`failed` entries in the first
    60 s of journal.

- `jonny5-spi-j5vr.service` — **not** restarted (IMU producer continues
  writing to `/dev/shm/j5vr_telemetry.json` as before).

- All other services (`jonny5-https`, `jonny5-https-443-proxy`,
  `jonny5-mediamtx`) — untouched.

## Final service state

```text
● jonny5-ws-teleop.service - JONNY5 WS-TELEOP (porta 8557)
     Loaded: loaded (/etc/systemd/system/jonny5-ws-teleop.service; enabled; preset: enabled)
     Active: active (running) since Tue 2026-04-21 19:06:56 CEST; 3s ago
 Invocation: 9b1e1199b9fc46b980d3992346b7edb5
   Main PID: 12572 (python3)
      Tasks: 6 (limit: 9570)
        CPU: 1.457s
```

`systemctl is-active jonny5-ws-teleop.service` → `active` (rechecked
after a further 8 s — still `active`, no error-scan hits).

## Telemetry path — final

- Producer (SPI bridge, `teleop/shared_state.py` default):
  `/dev/shm/j5vr_telemetry.json` — fresh, 786 bytes, mtime advancing.
- Consumer (`ws_server.py`, `web_services/shared_state.py`):
  resolved at runtime via `os.environ["J5VR_TELEMETRY_FILE"]` =
  `/dev/shm/j5vr_telemetry.json` — confirmed from `/proc/12572/environ`
  **and** via a read probe that imported `web_services/shared_state`
  with the same env and got back live telemetry:

  ```text
  consumer _TELEMETRY_FILE = /dev/shm/j5vr_telemetry.json
  TELEMETRY: OK, keys visible = ['servo_deg_B', 'servo_deg_S', 'servo_deg_G', 'imu_q_w', 'imu_valid']
  sample: {'servo_deg_B': 100, 'servo_deg_S': 88, 'servo_deg_G': 93,
           'imu_q_w': 0.97253..., 'imu_valid': True}
  ```

- `/tmp/j5vr_telemetry.json` — still absent, as expected. The consumer
  no longer tries to open it (env-var override takes precedence over
  the `/tmp/...` default in `shared_state.py:28`).

## assistMode — final

`/home/jonny5/raspberry5/config_runtime/robot/routing_config.json`
readback at the end of this session:

```text
assistMode = rate
```

No writes to `routing_config.json` were performed in this session.
`assistDls` sub-block values untouched (still the operator's stored
aggressive set — left alone per task rule "Non modificare assistDls").

## WebSocket clients on :8557

One external client observed continuously on port 8557:

| Observation | Peer | Port | Context |
|---|---|---|---|
| Pre-restart (entry of this session) | `192.168.10.80` | `58374` | Holdover from prior session |
| Post-restart | `192.168.10.80` | `54238` | Auto-reconnected with a new ephemeral port |

The client was disconnected for a few seconds during the service
restart and reconnected immediately afterwards — consistent with a
browser-side dashboard with auto-reconnect. The peer IP
`192.168.10.80` was not positively identified; this is very likely
Alessandro's own workstation / dashboard session.

Per task rule *"Se c'è un client esterno, NON fare test di movimento"*
— no motion test was attempted. The restart itself did not generate
any motion (ws_server on startup only queries STM32 status and syncs
config; it does not issue `SETPOSE` or any motion opcode).

## Readiness for next DLS motion test

**Ready** from a deployment-wiring standpoint:

- Hardening code deployed and `py_compile`-clean on the Pi.
- Telemetry bridge aligned: ws_server reads live IMU + servo data.
- `assistMode = rate` persistent default.
- Physical backup trail present for both the systemd unit and all
  replaced source files.

**Still conditional on operator checks** before the next motion test:

1. Confirm the external client on `192.168.10.80` is the operator's
   own dashboard and not an unattended session. If unknown, close it
   before enabling DLS.
2. Arm operator-side E-stop within reach.
3. Before flipping `assistMode=dls`, decide whether to apply the
   task-mandated conservative `assistDls` values (`gainM=0.08`,
   `lambdaMax=0.12`, `manipThresh=5e-4`, `maxDqDegPerTick=2.0`,
   `maxDxMmPerTick=15.0`, `nullSpaceGain=0.08`) — the stored
   aggressive values (`gainM=0.25` etc.) should NOT be used for
   first contact. The conservative set can be pushed via the
   smoke test's own `test_cfg["assistDls"] = {...}` literal (edit
   in place before running; hardened copy already has the physical
   backup guardrail), or by typing the values in the updated
   dashboard pane and saving.
4. When motion is enabled, the first `_process_head_assist_mode`
   tick will now emit:
   ```
   [HEAD-ASSIST] assistMode transition: None -> rate
   ```
   followed by `rate -> dls` when DLS is enabled. Look for these
   lines in `journalctl -u jonny5-ws-teleop -f` as the pre-test
   confirmation artifact mandated by the task.

No motion was issued, no firmware touched, no STM32 flashed, no
routing_config mutation. The system is in a clean `rate`-default
state with functional telemetry and hardened ws_server, ready for
a subsequent controlled motion test.
