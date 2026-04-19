# JONNY5-4.0 — VR Teleoperated Robot Arm (SLIM Workbench)

Snapshot of the JONNY5-4.0 teleoperated 6-DoF robot arm system used for the thesis.
This is the reduced "slim" working tree containing the controller, firmware, dashboard,
and audit scripts needed for external review.

## Architecture (3 tiers)

```
                    ┌───────────────────────────────┐
                    │  Quest 3 Headset / Dashboard   │
                    │   VR pose, intent, dashboard   │
                    └──────────────┬────────────────┘
                                   │ WebSocket (wss://…:8557)
                                   │ HTTPS (…:8443)
                    ┌──────────────▼────────────────┐
                    │   Raspberry Pi 5 (Pi-side)     │
                    │   WS server, IK/FK solver,     │
                    │   ASSIST logic, SPI master,    │
                    │   compare panel observers      │
                    └──────────────┬────────────────┘
                                   │ SPI1 @ 100 Hz (64-byte J5VR frames)
                    ┌──────────────▼────────────────┐
                    │   STM32 Nucleo-F446RE (Zephyr) │
                    │   1 kHz RT actuation loop,     │
                    │   servo PWM, IMU BNO085,       │
                    │   safety gating (deadman,      │
                    │   heartbeat, servo limits)     │
                    └───────────────────────────────┘
```

## Directory layout

| Path | Contents |
|---|---|
| `firmware/stm32/` | Zephyr firmware for STM32 Nucleo-F446RE. IMU (BNO085 via SHTP), servo actuation, HEAD wrist pipeline, SPI J5VR slave, safety |
| `raspberry/controller/` | Python control-plane: WS handlers, IK/FK solver (POE model), ASSIST head-follow logic, SPI data-plane bridge, settings manager, IMU analytics / validators |
| `raspberry/config_runtime/` | Runtime configs (robot offsets/limits, POE params, IMU calibration, TLS placeholders) |
| `raspberry/systemd/` | Service units for Pi daemons |
| `web/dashboard/` | Operator dashboard (FK/IK/IMU compare, joints, settings, VR config) |
| `web/vr/` | VR viewer (stereo WebRTC pipeline) |
| `scripts/` | Helper scripts (video pipeline, mediamtx) |
| `verify_*.py`, `analyze_*.py` | Audit/test scripts used during thesis validation (see below) |
| `deploy.sh`, `deploy.bat` | rsync+ssh deploy to Pi (default `jonny5@10.42.0.1`) |
| `setup_pi.sh` | One-time Pi bootstrap |

## Key subsystems

### IMU pipeline
- **BNO085** on SPI/I2C via SH-2 protocol, Rotation Vector @ ~100 Hz (`firmware/stm32/src/imu/bno085.[ch]`, `imu.c`)
- Frame alignment chain: `R_ee = R_home⁻¹ · R_world_bias⁻¹ · R_imu · R_mount⁻¹`
  - `imu_ee_mount.json`: mechanical chip-to-EE rotation
  - `imu_world_bias.json`: BNO085 magnetometer-derived yaw reference
  - `imu_home_ref.json`: operator-captured "Zero at HOME" offset
- Validators in `raspberry/controller/imu_analytics/`

### IK / FK
- POE (Product of Exponentials) forward/inverse kinematics
- `ik_solver.py`: scipy least-squares on 6-DoF residuals
- POE parameters in `config_runtime/kinematics/j5_poe_params.json`

### ASSIST mode (mode 5)
- Rate-based head-follow control for arm (B/S/G)
- Position-based wrist (Y/P/R) via firmware HEAD pipeline (relative quat)
- `head_assist.py`: remap logic, Jacobian local, reach scaling with cap
- Config overrides in `routing_config.json → headAssist`

### J5VR wire protocol
- 64-byte frame, little-endian payload
- mode=5 extension (marker 'I') carries arm targets B/S/G as int16 centi-degrees
- Header, sequence counter, CRC validated on both sides

## Audit scripts (all read-only, no firmware touch)

Run on Pi via `.venv/bin/python3`:

- `verify_joints_edges.py` — slider limits virtual↔physical validation
- `verify_zero_at_home.py` — end-to-end IMU zero-at-HOME experiment
- `verify_imu_alignment.py` / `verify_imu_stress.py` — IMU calibration & yaw-drift characterization
- `verify_ik_imu_compare.py` / `verify_ik_imu_frame*.py` — FK/IK/IMU compare pipeline checks
- `verify_assist_mode.py`, `verify_assist_extremes.py`, `verify_assist_audit.py` — ASSIST dynamics A/B tests
- `verify_isolated_joints.py`, `verify_elbow_*.py` — kinematic verification of joint cooperation
- `verify_vertical_tuning.py` — B1/B2/B3 vertical gain A/B/C/D experiment
- `analyze_pitch_direction.py` — offline CSV analysis of pitch direction from existing audit data

Each script outputs CSV + metrics JSON and prints a structured summary.

## Runtime services (Pi)

```
jonny5-ws-teleop       — WS server @ 8557, dispatcher, IK/FK, ASSIST
jonny5-spi-j5vr        — SPI master (Pi → STM32), 100 Hz J5VR frames
jonny5-https           — Static files + dashboard HTTPS @ 8443
jonny5-https-443-proxy — 443 forwarder for Quest browser compat
jonny5-captive-portal  — 10.42.0.1 captive portal for first-time Quest setup
jonny5-mediamtx        — WebRTC media server for stereo video
```

## Deploy

```bash
./deploy.sh jonny5@<pi-host>   # default jonny5@10.42.0.1
```

rsync's `raspberry/`, `web/`, `scripts/` to `/home/jonny5/raspberry5/`, installs
systemd services, generates TLS certs if absent, restarts services.

## What's new vs prior snapshots

This snapshot covers the BNO085 migration + ASSIST audit cycle:
- BNO085 driver (SHTP Rotation Vector) replacing MPU6050
- Zero-at-HOME correction layer for magnetometer drift
- Reach-scale cap (B1 tuning, `headMotionGainPitch=2.0` in runtime config)
- Compare page FK/IK/IMU in base frame with mount+world_bias+home chain
- Full audit of ASSIST yaw/pitch dynamics across 4 canonical poses
- Documented structural limits (rate-based lag, CENTRAL Jacobian rank deficiency)

## External analysis notes

For a reviewer: the control-plane entry point is `raspberry/controller/web_services/ws_server.py`
and the ASSIST logic is in `head_assist.py`. Firmware dispatch is `firmware/stm32/src/core/rt_loop.c`
and `firmware/stm32/src/servo/j5vr_actuation.c`. Runtime authoritative config is
`raspberry/config_runtime/robot/routing_config.json`.
