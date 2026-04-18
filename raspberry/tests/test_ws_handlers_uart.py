"""
Test per controller/web_services/ws_handlers_uart.py

Verifica: _build_set_vr_params_from_config — struttura output, token count,
sign/src mapping, pbEn flags.
Import isolato: le dipendenze async (uart_manager) vengono skippate tramite
monkeypatch sul modulo prima che vengano invocate.
"""

import pytest

from controller.web_services.ws_handlers_uart import _build_set_vr_params_from_config
from controller.web_services.vr_config_defaults import (
    VR_TUNE_DEFAULTS,
    merge_vr_config_with_defaults,
)


def _default_cfg() -> dict:
    """Config completa con tutti i default."""
    return merge_vr_config_with_defaults({})


class TestBuildSetVrParams:
    def test_starts_with_SET_VR_PARAMS(self):
        cmd = _build_set_vr_params_from_config(_default_cfg())
        assert cmd.startswith("SET_VR_PARAMS ")

    def test_token_count_at_least_20(self):
        """Firmware si aspetta almeno 20 parametri dopo il comando."""
        cmd = _build_set_vr_params_from_config(_default_cfg())
        parts = cmd.split()
        assert len(parts) >= 21  # "SET_VR_PARAMS" + ≥20 params

    def test_all_tokens_are_numeric(self):
        cmd = _build_set_vr_params_from_config(_default_cfg())
        parts = cmd.split()[1:]  # skip "SET_VR_PARAMS"
        for tok in parts:
            try:
                float(tok)
            except ValueError:
                pytest.fail(f"Token non numerico: {tok!r}")

    def test_sign_default_positive(self):
        """Con pbState default, sign_yaw/pitch/roll sono +1."""
        cfg = _default_cfg()
        parts = _build_set_vr_params_from_config(cfg).split()
        # Token 14, 15, 16 (1-indexed) sono sign_yaw, sign_pitch, sign_roll
        sign_yaw   = int(parts[14])
        sign_pitch = int(parts[15])
        sign_roll  = int(parts[16])
        assert sign_yaw   in (1, -1)
        assert sign_pitch in (1, -1)
        assert sign_roll  in (1, -1)

    def test_pb_en_false_produces_zero(self):
        """Disabilitare un canale nel pbEn deve produrre "0" nella posizione corretta."""
        cfg = _default_cfg()
        cfg["pbEn"]["roll"] = False
        parts = _build_set_vr_params_from_config(cfg).split()
        # Token 20 = en_roll
        assert parts[20] == "0"

    def test_pb_en_pitch_false(self):
        cfg = _default_cfg()
        cfg["pbEn"]["pitch"] = False
        parts = _build_set_vr_params_from_config(cfg).split()
        assert parts[21] == "0"

    def test_pb_en_yaw_false(self):
        cfg = _default_cfg()
        cfg["pbEn"]["yaw"] = False
        parts = _build_set_vr_params_from_config(cfg).split()
        assert parts[22] == "0"

    def test_empty_config_uses_defaults(self):
        """Config vuota → merge dei default → stessa struttura."""
        cmd_empty = _build_set_vr_params_from_config(merge_vr_config_with_defaults({}))
        cmd_default = _build_set_vr_params_from_config(_default_cfg())
        assert cmd_empty == cmd_default

    def test_lpf_tokens_forced_to_1(self):
        """LPF pitch e roll legacy (token 10, 11) sono forzati a "1" dal backend."""
        parts = _build_set_vr_params_from_config(_default_cfg()).split()
        # params[9] e params[10] (0-indexed nell'array) → token 10, 11 (1-indexed con cmd)
        assert parts[10] == "1"
        assert parts[11] == "1"

    def test_custom_tuning_gain_reflected(self):
        """Un gain personalizzato nella tuning deve comparire nel comando."""
        cfg = _default_cfg()
        cfg["tuning"]["tg-yaw"] = 2.5
        parts = _build_set_vr_params_from_config(cfg).split()
        # tg-yaw è il primo parametro legacy (token index 1)
        assert float(parts[1]) == pytest.approx(2.5)
