#!/bin/bash
# JONNY5 — Avvia MediaMTX con config/runtime/video/mediamtx.yml (unica baseline low-latency).
# Usato da jonny5-mediamtx.service. Richiede video_pipeline: webrtc in video_pipeline.yaml.
set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME_VIDEO_DIR="$REPO_ROOT/config/runtime/video"
CONFIG="$RUNTIME_VIDEO_DIR/video_pipeline.yaml"
MEDIAMTX_YML="$RUNTIME_VIDEO_DIR/mediamtx.yml"

if [ ! -f "$CONFIG" ]; then
  echo "ERR: video_pipeline config non trovata (required): $CONFIG" >&2
  exit 1
fi
if grep -q "video_pipeline: mjpeg" "$CONFIG"; then
  echo "ERR: video_pipeline è mjpeg; stack operativo è solo low-latency (webrtc). Imposta video_pipeline: webrtc in $CONFIG" >&2
  exit 1
fi
if ! grep -q "video_pipeline: webrtc" "$CONFIG"; then
  echo "ERR: richiesto video_pipeline: webrtc in $CONFIG" >&2
  exit 1
fi

if [ ! -f "$MEDIAMTX_YML" ]; then
  echo "ERR: MediaMTX config non trovato (required): $MEDIAMTX_YML" >&2
  exit 1
fi
echo "Using MediaMTX config: $MEDIAMTX_YML"

MEDIAMTX_BIN=""
if [ -x "/home/jonny5/mediamtx" ]; then
  MEDIAMTX_BIN="/home/jonny5/mediamtx"
elif [ -x "/home/jonny5/mediamtx/mediamtx" ]; then
  MEDIAMTX_BIN="/home/jonny5/mediamtx/mediamtx"
elif command -v mediamtx >/dev/null 2>&1; then
  MEDIAMTX_BIN="mediamtx"
else
  echo "ERR: MediaMTX non trovato." >&2
  exit 1
fi

exec "$MEDIAMTX_BIN" "$MEDIAMTX_YML"
