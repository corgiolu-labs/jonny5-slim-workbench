"""
Test per controller/spi_dataplane/j5vr_frame.py

Verifica: header bytes, payload layout, build_setpoint_frame roundtrip,
mode handling, camctrl, mode5 arm extension.
"""

import struct

import pytest

from controller.spi_dataplane.j5vr_frame import (
    J5VRFrame,
    J5VRPayload,
    build_setpoint_frame,
    J5_PROTOCOL_FRAME_SIZE,
    J5_FRAME_TYPE_J5VR,
    J5_FRAME_TYPE_TELEMETRY,
    _norm_float_to_i16,
    _norm_float_to_u8_intensity,
    _angle_deg_to_cdeg_i16,
)


class TestNormHelpers:
    def test_float_to_i16_positive(self):
        assert _norm_float_to_i16(1.0) == 32767

    def test_float_to_i16_negative(self):
        # Scaling uses *32767 (symmetric), so -1.0 → -32767, not -32768
        assert _norm_float_to_i16(-1.0) == -32767

    def test_float_to_i16_zero(self):
        assert _norm_float_to_i16(0.0) == 0

    def test_float_to_i16_passthrough_large_int(self):
        assert _norm_float_to_i16(1000) == 1000

    def test_float_to_i16_clamp_over(self):
        assert _norm_float_to_i16(2.0) == 32767

    def test_float_to_i16_clamp_under(self):
        # Clamps to max(-32768, -32767) = -32767 (symmetric design)
        assert _norm_float_to_i16(-2.0) == -32767

    def test_u8_intensity_float_max(self):
        assert _norm_float_to_u8_intensity(1.0) == 255

    def test_u8_intensity_float_zero(self):
        assert _norm_float_to_u8_intensity(0.0) == 0

    def test_u8_intensity_passthrough_int(self):
        assert _norm_float_to_u8_intensity(128) == 128

    def test_angle_deg_to_cdeg(self):
        assert _angle_deg_to_cdeg_i16(90.0) == 9000
        assert _angle_deg_to_cdeg_i16(0.0) == 0
        assert _angle_deg_to_cdeg_i16(180.0) == 18000


class TestJ5VRPayloadBytes:
    def test_length_54(self):
        p = J5VRPayload()
        assert len(p.to_bytes()) == 54

    def test_mode_at_offset_0(self):
        p = J5VRPayload(mode=3)
        b = p.to_bytes()
        assert b[0] == 3

    def test_joy_x_big_endian(self):
        p = J5VRPayload(joy_x=256)
        b = p.to_bytes()
        val = struct.unpack_from(">h", b, 1)[0]
        assert val == 256

    def test_joy_y_big_endian(self):
        p = J5VRPayload(joy_y=-100)
        b = p.to_bytes()
        val = struct.unpack_from(">h", b, 3)[0]
        assert val == -100

    def test_intensity_offset_9(self):
        p = J5VRPayload(intensity=200)
        b = p.to_bytes()
        assert b[9] == 200

    def test_grip_offset_10(self):
        p = J5VRPayload(grip=1)
        b = p.to_bytes()
        assert b[10] == 1

    def test_heartbeat_offset_11(self):
        p = J5VRPayload(vr_heartbeat=1000)
        b = p.to_bytes()
        val = struct.unpack_from(">H", b, 11)[0]
        assert val == 1000

    def test_quat_w_offset_16(self):
        p = J5VRPayload(quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)
        b = p.to_bytes()
        w = struct.unpack_from(">f", b, 16)[0]
        assert abs(w - 1.0) < 1e-6

    def test_buttons_left_offset_32(self):
        p = J5VRPayload(buttons_left=0xAB)
        b = p.to_bytes()
        val = struct.unpack_from(">H", b, 32)[0]
        assert val == 0xAB

    def test_camctrl_marker_C(self):
        p = J5VRPayload(camctrl_cmd=1, camctrl_delta=50)
        b = p.to_bytes()
        assert b[36] == ord("C")
        assert b[37] == 1
        delta = struct.unpack_from(">h", b, 38)[0]
        assert delta == 50

    def test_mode5_arm_marker_I(self):
        p = J5VRPayload(
            mode=5,
            mode5_arm_valid=1,
            mode5_control_flags=0,
            mode5_target_id=7,
            mode5_base_deg=90.0,
            mode5_spalla_deg=45.0,
            mode5_gomito_deg=135.0,
        )
        b = p.to_bytes()
        assert b[36] == ord("I")
        tid = struct.unpack_from(">H", b, 38)[0]
        assert tid == 7
        base = struct.unpack_from(">h", b, 40)[0]
        assert base == 9000  # 90 * 100
        spalla = struct.unpack_from(">h", b, 42)[0]
        assert spalla == 4500  # 45 * 100


class TestJ5VRFrameBytes:
    def test_total_length_64(self):
        frame = J5VRFrame()
        assert len(frame.to_bytes()) == 64

    def test_header_J5(self):
        b = J5VRFrame().to_bytes()
        assert b[0] == ord("J")
        assert b[1] == ord("5")

    def test_protocol_version_1(self):
        b = J5VRFrame().to_bytes()
        assert b[2] == 1

    def test_frame_type_j5vr(self):
        b = J5VRFrame().to_bytes()
        assert b[3] == J5_FRAME_TYPE_J5VR

    def test_sequence_counter_big_endian(self):
        b = J5VRFrame(sequence_counter=0x0102).to_bytes()
        assert b[4] == 0x01
        assert b[5] == 0x02

    def test_payload_len_byte_6(self):
        b = J5VRFrame().to_bytes()
        assert b[6] == 64

    def test_payload_embedded_at_offset_8(self):
        p = J5VRPayload(mode=7)
        b = J5VRFrame(payload=p).to_bytes()
        assert b[8] == 7  # mode is first byte of payload

    def test_frame_type_telemetry(self):
        b = J5VRFrame(frame_type=J5_FRAME_TYPE_TELEMETRY).to_bytes()
        assert b[3] == J5_FRAME_TYPE_TELEMETRY


class TestBuildSetpointFrame:
    def test_basic_mode_int(self):
        frame = build_setpoint_frame({"mode": 1})
        b = frame.to_bytes()
        assert b[8] == 1  # payload[0] = mode

    def test_mode_legacy_string(self):
        frame = build_setpoint_frame({"mode": "RELATIVE_MOVE"})
        b = frame.to_bytes()
        assert b[8] == 1

    def test_mode_idle_string(self):
        frame = build_setpoint_frame({"mode": "IDLE"})
        b = frame.to_bytes()
        assert b[8] == 0

    def test_mode_3_zeros_joystick(self):
        frame = build_setpoint_frame({"mode": 3, "joy_x": 0.5, "joy_y": -0.5})
        b = frame.to_bytes()
        joy_x = struct.unpack_from(">h", b, 9)[0]
        joy_y = struct.unpack_from(">h", b, 11)[0]
        assert joy_x == 0
        assert joy_y == 0

    def test_mode_1_preserves_joystick(self):
        frame = build_setpoint_frame({"mode": 1, "joy_x": 1.0})
        b = frame.to_bytes()
        joy_x = struct.unpack_from(">h", b, 9)[0]
        assert joy_x == 32767

    def test_heartbeat_roundtrip(self):
        frame = build_setpoint_frame({"mode": 0, "heartbeat": 42})
        b = frame.to_bytes()
        hb = struct.unpack_from(">H", b, 19)[0]  # offset 8+11 = 19
        assert hb == 42

    def test_grip_roundtrip(self):
        frame = build_setpoint_frame({"mode": 1, "grip": 1})
        b = frame.to_bytes()
        assert b[18] == 1  # offset 8+10 = 18

    def test_camctrl_dict(self):
        frame = build_setpoint_frame({
            "mode": 0,
            "camctrl": {"cmd": "zoom", "delta": -30},
        })
        b = frame.to_bytes()
        assert b[44] == ord("C")   # offset 8+36 = 44
        assert b[45] == 2          # zoom = 2
        delta = struct.unpack_from(">h", b, 46)[0]
        assert delta == -30

    def test_frame_size_always_64(self):
        for mode in range(6):
            frame = build_setpoint_frame({"mode": mode})
            assert len(frame.to_bytes()) == 64

    def test_sequence_counter_passed_through(self):
        frame = build_setpoint_frame({}, sequence_counter=0xCAFE)
        b = frame.to_bytes()
        seq = (b[4] << 8) | b[5]
        assert seq == 0xCAFE
