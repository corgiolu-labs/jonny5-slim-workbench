#!/usr/bin/env python3
"""
ws_server.py — WebSocket Teleoperation Server (WS-TELEOP), punto di ingresso.

Porta 8557. Riceve intent JSON dal browser WebXR, valida, converte in
struttura J5VR (senza inviare SPI).

Questo file contiene SOLO:
  - startup del server (main, SSL, task background)
  - handle_client: dispatcher messaggi → moduli ws_handlers_*
  - contatori di log locali (LOG_INTENT_EVERY_N, _intent_log_counter)

Tutta la logica è nei moduli:
  ws_core.py               — costanti, utility
  ws_handlers_intent.py    — intent VR, HEAD mode
  ws_handlers_settings.py  — impostazioni, calibrazione
  ws_handlers_uart.py      — comandi UART, demo, polling stato
  ws_handlers_imu.py       — telemetria IMU, feedback ACK

Nota: eseguire da root (raspberry5 sul Pi) con PYTHONPATH:
  PYTHONPATH=/home/jonny5/raspberry5 python3 controller/web_services/ws_server.py

[RPi-0.5] Ridotto a puro startup + dispatcher.
[RPi-0.6] Import ordinati PEP8 (std → third-party → local); docstring aggiornata.
[RPi-0.7] Accesso a _dashboard_head_active via getter pubblico
          _intent.get_dashboard_head_active() anziché accesso diretto.
[RPi-1.0] Chiamate a init_events() in main() per inizializzare asyncio.Event
          nei moduli handler dentro il loop attivo (no modulo load).
"""

# stdlib
import asyncio
import json
import logging
import os
import ssl
import time

# third-party
try:
    import websockets
except ImportError:
    raise SystemExit("Richiesto: pip install websockets")

# local
from controller.teleop import shared_state
from controller.uart import uart_manager
from controller.web_services import ws_handlers_intent as _intent
from controller.web_services import ws_handlers_imu as _imu
from controller.web_services import ws_handlers_poe as _poe
from controller.web_services import ws_handlers_kinematics as _kinematics
from controller.web_services import ws_handlers_settings as _settings
from controller.web_services import ws_handlers_uart as _uart
from controller.web_services.head_assist import (
    HeadAssistState,
    _quat_to_rpy_deg,
    parse_head_assist_cfg,
    step_mode5_head_assist,
)
from controller.web_services.head_assist_dls import (
    parse_assist_dls_cfg,
    step_dls_head_assist,
)
from controller.web_services import runtime_config_paths as rcfg
from controller.web_services.vr_config_defaults import merge_vr_config_with_defaults
from controller.web_services.ws_core import (
    clients,
)

# ---------------------------------------------------------------------------
# Configurazione
# ---------------------------------------------------------------------------
WS_PORT = 8557

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("ws_teleop")


def _resolve_tls_pair() -> tuple[str, str, str, str]:
    requested_cert = os.path.abspath(
        os.environ.get("WEBRTC_CERT_FILE", rcfg.get_runtime_config_path("tls_cert"))
    )
    requested_key = os.path.abspath(
        os.environ.get("WEBRTC_KEY_FILE", rcfg.get_runtime_config_path("tls_key"))
    )
    resolved_cert = rcfg.resolve_existing_config_path("tls_cert", env_var="WEBRTC_CERT_FILE")
    resolved_key = rcfg.resolve_existing_config_path("tls_key", env_var="WEBRTC_KEY_FILE")
    return requested_cert, requested_key, resolved_cert, resolved_key

# ---------------------------------------------------------------------------
# Stato locale del dispatcher
# ---------------------------------------------------------------------------
# Throttle log intent
LOG_INTENT_EVERY_N  = 20
_intent_log_counter = 0

# Contatore per log diagnostico 4 livelli (quaternioni visore → SPI)
_vr_input_log_count = 0


# ---------------------------------------------------------------------------
# MODE=5 — HEAD OVERFLOW ASSIST (ex IK MODE): B/S/G solo in overflow polso.
# Parametri: routing_config.json → merge_vr_config_with_defaults (headAssist).
# ---------------------------------------------------------------------------
_head_assist_state = HeadAssistState()
_dx_runtime_cfg_cache = {
    "path": rcfg.get_runtime_config_read_path("routing_config"),
    "mtime": None,
    "cfg": merge_vr_config_with_defaults({}),
}
_last_assist_mode_flag: str | None = None


def _extract_servo_physical_deg_from_telemetry() -> list[float] | None:
    telem = shared_state.read_telemetry_from_file()
    if not isinstance(telem, dict):
        return None
    keys = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")
    if any(k not in telem for k in keys):
        return None
    try:
        return [float(telem[k]) for k in keys]
    except Exception:
        return None


def _attach_mode5_arm_target(intent: dict, physical_angles: list[int] | None, grip_active: bool, hold_active: bool, target_id: int) -> None:
    if physical_angles is None or len(physical_angles) != 3:
        intent["mode5_arm"] = {
            "valid": False,
            "grip_active": bool(grip_active),
            "hold_active": bool(hold_active),
            "target_id": int(target_id) & 0xFFFF,
            "physical_deg": None,
        }
        return
    intent["mode5_arm"] = {
        "valid": True,
        "grip_active": bool(grip_active),
        "hold_active": bool(hold_active),
        "target_id": int(target_id) & 0xFFFF,
        "physical_deg": [int(v) for v in physical_angles[:3]],
    }


def _load_dx_runtime_cfg() -> dict:
    path = rcfg.get_runtime_config_read_path("routing_config")
    cached_path = _dx_runtime_cfg_cache.get("path")
    cached_mtime = _dx_runtime_cfg_cache.get("mtime")
    if path != cached_path:
        _dx_runtime_cfg_cache["path"] = path
        _dx_runtime_cfg_cache["mtime"] = None
        cached_mtime = None
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        mtime = None
    if mtime == cached_mtime and isinstance(_dx_runtime_cfg_cache.get("cfg"), dict):
        return _dx_runtime_cfg_cache["cfg"]
    raw_cfg = rcfg.load_runtime_json("routing_config", default={}) or {}
    merged = merge_vr_config_with_defaults(raw_cfg if isinstance(raw_cfg, dict) else {})
    _dx_runtime_cfg_cache["mtime"] = mtime
    _dx_runtime_cfg_cache["cfg"] = merged
    return merged


def _process_head_assist_mode(intent: dict) -> bool:
    """
    Mode=5 — HEAD OVERFLOW ASSIST:
      deadman classico a doppio grip;
      B/S/G da telemetria + correzione morbida se yaw/pitch/roll polso in warning/critico;
      polso continua sulla pipeline HEAD del firmware.
    """
    if int(intent.get("mode", -1)) != 5:
        return False

    st = _head_assist_state
    cfg = _load_dx_runtime_cfg()
    ha = parse_head_assist_cfg(cfg.get("headAssist") or {})

    if not ha["enabled"]:
        _attach_mode5_arm_target(intent, None, grip_active=False, hold_active=False, target_id=st.target_id)
        st.filt_b = st.filt_s = st.filt_g = None
        st.last_arm_physical = None
        st.last_ts = 0.0
        return True

    now = float(intent.get("timestamp", time.monotonic()))
    grip = int(intent.get("grip", 0)) == 1
    head_quat = None
    head_rpy = None
    try:
        qw = float(intent.get("quat_w", 1.0))
        qx = float(intent.get("quat_x", 0.0))
        qy = float(intent.get("quat_y", 0.0))
        qz = float(intent.get("quat_z", 0.0))
        head_quat = (qw, qx, qy, qz)
        head_rpy = _quat_to_rpy_deg(qw, qx, qy, qz)
    except Exception:
        head_quat = None
        head_rpy = None

    servo_physical = _extract_servo_physical_deg_from_telemetry()
    phys_list: list[float] | None = (
        [float(x) for x in servo_physical] if servo_physical else None
    )

    assist_mode_flag = str(cfg.get("assistMode", "rate")).strip().lower()
    global _last_assist_mode_flag
    if assist_mode_flag != _last_assist_mode_flag:
        logger.info("[HEAD-ASSIST] assistMode transition: %s -> %s",
                    _last_assist_mode_flag, assist_mode_flag)
        _last_assist_mode_flag = assist_mode_flag
    if assist_mode_flag == "dls":
        ha_dls = parse_assist_dls_cfg(cfg.get("assistDls") or {})
        arm, g_active, hold, tid = step_dls_head_assist(
            raw_grip_active=grip,
            physical_six=phys_list,
            head_quat_wxyz=head_quat,
            limits_src=cfg.get("limits") or {},
            ha=ha,
            ha_dls=ha_dls,
            state=st,
            now=now,
        )
    else:
        arm, g_active, hold, tid = step_mode5_head_assist(
            raw_grip_active=grip,
            physical_six=phys_list,
            head_rpy_deg=head_rpy,
            limits_src=cfg.get("limits") or {},
            ha=ha,
            state=st,
            now=now,
        )

    if grip and phys_list is None:
        logger.warning("[HEAD-ASSIST] grip attivo ma telemetria servo assente")
        _attach_mode5_arm_target(intent, arm, grip_active=False, hold_active=hold, target_id=tid)
        return True

    _attach_mode5_arm_target(intent, arm, grip_active=g_active, hold_active=hold, target_id=tid)
    return True


# ---------------------------------------------------------------------------
# handle_client — dispatcher principale
# ---------------------------------------------------------------------------

async def handle_client(websocket, path=None):  # path è opzionale nelle nuove versioni di websockets
    remote = getattr(websocket, "remote_address", "?") or "?"
    logger.info("Client connesso: %s (path: %s)", remote, path or "/")
    clients.add(websocket)
    try:
        # Log informazioni sulla connessione SSL se disponibili
        if hasattr(websocket, "secure") and websocket.secure:
            logger.info("Connessione WSS (SSL/TLS) da %s", remote)
        else:
            logger.info("Connessione WS (non crittografata) da %s", remote)

        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as e:
                logger.warning("JSON non valido: %s", e)
                continue

            # Comando controllo IMU (UART): non è un intent
            if isinstance(data, dict) and data.get("type") == "set_imu":
                await _uart.handle_set_imu(websocket, data)
                continue

            # Cambio modalità VR dalla dashboard (set_vr_mode)
            if isinstance(data, dict) and data.get("type") == "set_vr_mode":
                await _intent.handle_set_vr_mode(websocket, data)
                continue

            # Lettura impostazioni
            if isinstance(data, dict) and data.get("type") == "get_settings":
                await _settings.handle_get_settings(websocket)
                continue

            # Salvataggio impostazioni
            if isinstance(data, dict) and data.get("type") == "save_settings":
                await _settings.handle_save_settings(websocket, data)
                continue

            # Parametri POE — persistenza lato Raspberry (source of truth)
            if isinstance(data, dict) and data.get("type") == "get_poe_params":
                await _poe.handle_get_poe_params(websocket)
                continue

            if isinstance(data, dict) and data.get("type") == "set_poe_params":
                await _poe.handle_set_poe_params(websocket, data)
                continue

            # FK POE (dashboard): stesso modello della IK runtime
            if isinstance(data, dict) and data.get("type") == "compute_fk_poe":
                await _kinematics.handle_compute_fk_poe(websocket, data)
                continue

            # Calibrazione stereo: broadcast a tutti i client connessi (visore + dashboard)
            if isinstance(data, dict) and data.get("type") == "vr_calib":
                await _settings.handle_vr_calib(websocket, data)
                continue

            # Applicazione offset meccanici al firmware via SET_OFFSETS UART
            if isinstance(data, dict) and data.get("type") == "apply_offsets":
                await _settings.handle_apply_offsets(websocket, data)
                continue

            # Restituisce la configurazione PWM persistita.
            if isinstance(data, dict) and data.get("type") == "get_pwm_config":
                cfg = _uart.load_persisted_pwm_config()
                try:
                    await websocket.send(json.dumps({"type": "pwm_config", "config": cfg}))
                except Exception:
                    pass
                continue

            # Salva e applica configurazione PWM servo al firmware.
            if isinstance(data, dict) and data.get("type") == "save_pwm_config":
                cfg = data.get("config", {})
                saved = _uart.save_pwm_config(cfg)
                ok = await _uart.apply_pwm_config_now(cfg)
                try:
                    await websocket.send(json.dumps({"type": "pwm_config_applied", "ok": ok, "saved": saved}))
                except Exception:
                    pass
                continue

            # Applica al firmware la config IMU-VR persistita su routing_config.json.
            if isinstance(data, dict) and data.get("type") == "apply_saved_vr_config":
                ok = await _uart.apply_persisted_vr_config_to_firmware()
                logger.info("[VR_CONFIG][APPLY] apply_saved_vr_config richiesto da client -> ok=%s", ok)
                try:
                    await websocket.send(json.dumps({"type": "vr_config_applied", "ok": ok}))
                except Exception:
                    pass
                continue

            # Comandi UART ENABLE / STOP / STATUS? / SAFE / RESET + HOME / PARK / TELEOPPOSE / SETPOSE
            if isinstance(data, dict) and data.get("type") == "uart":
                await _uart.handle_uart_cmd(websocket, data)
                continue

            if isinstance(data, dict) and data.get("type") == "self_test":
                await _uart.handle_self_test(websocket, data)
                continue

            # --- Intent VR ---
            global _intent_log_counter
            _intent_log_counter += 1
            if _intent_log_counter % LOG_INTENT_EVERY_N == 0:
                logger.info(
                    "Intent ricevuto: mode=%s joy_x=%s joy_y=%s pitch=%s yaw=%s intensity=%s buttons_L=0x%04x buttons_R=0x%04x camctrl=%s",
                    data.get("mode"),
                    data.get("joy_x"),
                    data.get("joy_y"),
                    data.get("pitch"),
                    data.get("yaw"),
                    data.get("intensity"),
                    int(data.get("buttons_left",  0) or 0) & 0xFFFF,
                    int(data.get("buttons_right", 0) or 0) & 0xFFFF,
                    data.get("camctrl"),
                )

            intent, err = _intent.validate_and_build_intent(data)
            if err:
                logger.warning("Validazione fallita: %s", err)
                continue

            # Mode=5: HEAD OVERFLOW ASSIST (config: routing_config headAssist).
            if int(intent.get("mode", -1)) == 5:
                _process_head_assist_mode(intent)
                # Mantieni comunque latest_intent aggiornato per telemetria/debug.
                with shared_state.lock:
                    shared_state.latest_intent = intent
                shared_state.write_intent_to_file(intent)
                continue

            # Log minimale throttled: include mode + heartbeat + camctrl (se presente)
            if _intent_log_counter % LOG_INTENT_EVERY_N == 0:
                if intent.get("camctrl") is not None:
                    logger.info("Intent validato: mode=%d hb=%d camctrl=%s",
                                intent["mode"], intent.get("heartbeat", 0), intent["camctrl_payload"])
                else:
                    logger.info("Intent validato: mode=%d hb=%d",
                                intent["mode"], intent.get("heartbeat", 0))

            # Se HEAD mode dashboard è attivo: preserva mode=3, grip e buttons dal task heartbeat,
            # ma aggiorna i quaternioni del visore (fondamentali per il closed loop HEAD).
            if _intent.get_dashboard_head_active() and int(intent.get("mode", -1)) in (3, 4, 5):
                # HEAD "sticky": non interrompere il loop su frame intent transitori
                # (race comune: un frame mode=2 subito dopo set_vr_mode=3).
                # L'uscita da HEAD deve avvenire in modo esplicito con set_vr_mode.
                intent["mode"]          = 3
                intent["grip"]          = 1
                intent["buttons_left"]  = 0x0002
                intent["buttons_right"] = 0x0002

            old_mode = -1
            with shared_state.lock:
                prev = shared_state.latest_intent
                if isinstance(prev, dict):
                    try:
                        old_mode = int(prev.get("mode", -1))
                    except (TypeError, ValueError):
                        old_mode = -1
                shared_state.latest_intent = intent
            shared_state.write_intent_to_file(intent)

            mode_i = int(intent.get("mode", -1))
            # Se arrivano intent MANUAL espliciti, interrompi eventuale heartbeat HEAD
            # che altrimenti riscrive buttons_left=0x0002 e contamina il controllo ROLL.
            if mode_i == 2:
                try:
                    _intent.stop_head_mode("auto-stop on manual intent")
                except Exception:
                    pass
            if mode_i in (3, 4, 5) and mode_i != old_mode:
                asyncio.create_task(_uart.apply_vr_config_on_intent_head_hybrid_entry(mode_i))

            # Log diagnostico 4 livelli: quaternione ricevuto dal visore (solo mode=3, ogni 5° messaggio)
            if intent.get("mode") == 3:
                global _vr_input_log_count
                _vr_input_log_count += 1
                if _vr_input_log_count % 5 == 0:
                    logger.info(
                        "[VR-INPUT] qvis=(%.3f %.3f %.3f %.3f)",
                        intent["quat_w"], intent["quat_x"], intent["quat_y"], intent["quat_z"],
                    )

    except websockets.exceptions.ConnectionClosed as e:
        logger.info("Client disconnesso: %s (code: %s, reason: %s)", remote, e.code, e.reason)
        raise
    except websockets.exceptions.InvalidMessage as e:
        logger.warning("Messaggio non valido da %s: %s", remote, e)
    except Exception as e:
        logger.exception("Errore gestione client %s: %s", remote, e)
    finally:
        try:
            clients.discard(websocket)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

async def main():
    # [RPi-1.0] Inizializza asyncio.Event nei moduli handler dentro il loop attivo.
    # Deve precedere qualsiasi uso degli Event (task, callback, coroutine).
    _intent.init_events()
    _uart.init_events()

    # Registra il loop asyncio nel modulo UART (per callback threadsafe)
    _uart.set_main_loop(asyncio.get_running_loop())

    # Registra callback per righe non-solicitate (SETPOSE_DONE)
    uart_manager.set_unsolicited_callback(_uart.on_uart_unsolicited)

    # Certificati: prefer runtime config, con fallback legacy durante la transizione.
    requested_cert, requested_key, cert_file, key_file = _resolve_tls_pair()
    ssl_ctx   = None
    if os.path.exists(cert_file) and os.path.exists(key_file):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
        if cert_file != requested_cert or key_file != requested_key:
            logger.warning(
                "TLS fallback attivo: richiesto=(%s, %s) risolto=(%s, %s)",
                requested_cert,
                requested_key,
                cert_file,
                key_file,
            )
        logger.info("SSL/WSS abilitato (cert: %s)", cert_file)
    else:
        logger.warning(
            "Certificati SSL non trovati (%s, %s) - solo WS supportato",
            requested_cert,
            requested_key,
        )

    proto = "wss" if ssl_ctx else "ws"
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT, ping_interval=20, ping_timeout=10, ssl=ssl_ctx):
        logger.info("WS-TELEOP in ascolto su %s://0.0.0.0:%s (nessun invio SPI)", proto, WS_PORT)
        asyncio.create_task(_imu.feedback_loop())
        asyncio.create_task(_imu.imu_debug_loop(_uart.get_robot_state))
        asyncio.create_task(_uart.startup_imuon())
        asyncio.create_task(_uart.robot_state_poll_loop())
        asyncio.create_task(_uart.apply_vr_config_at_startup())
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
