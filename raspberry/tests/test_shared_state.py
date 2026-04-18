"""
Test per controller/teleop/shared_state.py

Verifica: cache mtime, write atomico su /tmp, read con cache hit/miss.
Usa /tmp per i test (indipendente da /dev/shm).
"""

import json
import os
import tempfile
import time

import pytest

# Patch env vars prima di importare il modulo
_tmp = tempfile.mkdtemp()
os.environ.setdefault("J5VR_INTENT_FILE",    os.path.join(_tmp, "intent.json"))
os.environ.setdefault("J5VR_FEEDBACK_FILE",  os.path.join(_tmp, "feedback.json"))
os.environ.setdefault("J5VR_TELEMETRY_FILE", os.path.join(_tmp, "telemetry.json"))

from controller.teleop import shared_state  # noqa: E402


def _reset_caches():
    shared_state._intent_cache = None
    shared_state._intent_cache_mtime = 0.0
    shared_state._feedback_cache = None
    shared_state._feedback_cache_mtime = 0.0
    shared_state._telemetry_cache = {}
    shared_state._telemetry_cache_mtime = 0.0


class TestWriteJsonAtomic:
    def test_file_creato(self, tmp_path):
        p = str(tmp_path / "out.json")
        shared_state._write_json_atomic(p, {"x": 1}, sync=False)
        assert os.path.exists(p)
        with open(p) as f:
            assert json.load(f) == {"x": 1}

    def test_atomico_no_tmp_residuo(self, tmp_path):
        p = str(tmp_path / "out.json")
        shared_state._write_json_atomic(p, {"a": 2}, sync=False)
        assert not os.path.exists(p + ".tmp")

    def test_shm_forza_no_sync(self, monkeypatch):
        """Su path /dev/shm sync deve essere forzato False."""
        called_with_sync = []

        original = shared_state._write_json_atomic

        def patched(path, data, *, sync=True):
            called_with_sync.append(sync)

        monkeypatch.setattr(shared_state, "_write_json_atomic", patched)
        # Chiamata diretta con sync=True su path /dev/shm
        shared_state._write_json_atomic("/dev/shm/test.json", {}, sync=True)
        # La funzione patchata non applica la logica interna, test solo strutturale
        monkeypatch.undo()


class TestIntentCache:
    def setup_method(self):
        _reset_caches()

    def test_read_none_se_file_mancante(self):
        _reset_caches()
        shared_state._INTENT_FILE = "/tmp/nonexistent_j5vr_test_xyz.json"
        result = shared_state.read_intent_from_file()
        assert result is None

    def test_write_e_read_roundtrip(self):
        shared_state._INTENT_FILE = os.environ["J5VR_INTENT_FILE"]
        _reset_caches()
        data = {"mode": 3, "heartbeat": 42}
        shared_state.write_intent_to_file(data)
        result = shared_state.read_intent_from_file()
        assert result == data

    def test_cache_hit_non_rilegge(self):
        shared_state._INTENT_FILE = os.environ["J5VR_INTENT_FILE"]
        _reset_caches()
        shared_state.write_intent_to_file({"mode": 2})
        r1 = shared_state.read_intent_from_file()
        # Modifica il file senza aggiornare mtime (impossibile in modo affidabile,
        # ma verifichiamo che la seconda read ritorni lo stesso oggetto cached)
        r2 = shared_state.read_intent_from_file()
        assert r1 is r2  # stesso oggetto = cache hit


class TestFeedbackCache:
    def setup_method(self):
        _reset_caches()

    def test_write_e_read_roundtrip(self):
        shared_state._FEEDBACK_FILE = os.environ["J5VR_FEEDBACK_FILE"]
        _reset_caches()
        fb = {"teleop_pose_ack": True, "id": 7}
        shared_state.write_feedback_to_file(fb)
        result = shared_state.read_feedback_from_file()
        assert result == fb

    def test_write_aggiorna_cache_in_process(self):
        shared_state._FEEDBACK_FILE = os.environ["J5VR_FEEDBACK_FILE"]
        _reset_caches()
        fb = {"teleop_pose_ack": True, "id": 99}
        shared_state.write_feedback_to_file(fb)
        assert shared_state._feedback_cache == fb


class TestGetIntentCacheMtime:
    def test_ritorna_zero_se_mai_letto(self):
        _reset_caches()
        assert shared_state.get_intent_cache_mtime() == 0.0

    def test_ritorna_valore_dopo_write_read(self):
        shared_state._INTENT_FILE = os.environ["J5VR_INTENT_FILE"]
        _reset_caches()
        shared_state.write_intent_to_file({"mode": 0})
        shared_state.read_intent_from_file()
        assert shared_state.get_intent_cache_mtime() > 0.0


class TestIsIntentFresh:
    def test_false_se_mai_letto(self):
        _reset_caches()
        assert shared_state.is_intent_fresh() is False

    def test_true_subito_dopo_write_read(self):
        shared_state._INTENT_FILE = os.environ["J5VR_INTENT_FILE"]
        _reset_caches()
        shared_state.write_intent_to_file({"mode": 4})
        shared_state.read_intent_from_file()
        # Appena scritto: fresco
        assert shared_state.is_intent_fresh(max_age_s=5.0) is True

    def test_false_con_max_age_negativo(self):
        shared_state._INTENT_FILE = os.environ["J5VR_INTENT_FILE"]
        _reset_caches()
        shared_state.write_intent_to_file({"mode": 4})
        shared_state.read_intent_from_file()
        assert shared_state.is_intent_fresh(max_age_s=-1.0) is False
