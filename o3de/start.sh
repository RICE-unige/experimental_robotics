#!/bin/bash
set -euo pipefail

PROJECT_ROOT=${PROJECT_ROOT:-/data/workspace/ROSbotXLDemo}
BIN_DIR=${PROJECT_BIN_DIR:-$PROJECT_ROOT/build/linux/bin/profile}
ASSET_PROCESSOR_BIN=${ASSET_PROCESSOR_BIN:-$BIN_DIR/AssetProcessor}

VNC_DISPLAY=${VNC_DISPLAY:-:1}
VNC_PORT=${VNC_PORT:-5901}
VNC_RESOLUTION=${VNC_RESOLUTION:-1920x1080}
VNC_DEPTH=${VNC_DEPTH:-24}
VGL_DISPLAY=${VGL_DISPLAY:-:0}
VNC_PASSWORD=${VNC_PASSWORD:-}
O3DE_EXECUTABLE=${O3DE_EXECUTABLE:-Editor}
O3DE_ARGS=${O3DE_ARGS:-}
O3DE_ASSET_PROCESSOR=${O3DE_ASSET_PROCESSOR:-1}

TARGET_BIN="$BIN_DIR/$O3DE_EXECUTABLE"

log() {
    printf '[o3de] %s\n' "$*"
}

fatal() {
    printf '[o3de] ERROR: %s\n' "$*" >&2
    exit 1
}

cleanup() {
    set +e
    for pid in ${PIDS:-}; do
        if kill -0 "$pid" >/dev/null 2>&1; then
            kill "$pid" >/dev/null 2>&1 || true
        fi
    done
    wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ensure_binary() {
    local binary_path="$1"
    [ -x "$binary_path" ] || fatal "Expected executable not found: $binary_path"
}

ensure_binary "$TARGET_BIN"

command -v vglrun >/dev/null 2>&1 || fatal "VirtualGL (vglrun) not installed"
command -v Xvfb >/dev/null 2>&1 || fatal "Xvfb not installed"
command -v x11vnc >/dev/null 2>&1 || fatal "x11vnc not installed"

if command -v vglserver_config >/dev/null 2>&1; then
    # Configure VirtualGL environment quietly; failure is non-fatal but logged
    if ! vglserver_config -config +s +f -t >/tmp/vglserver_config.log 2>&1; then
        log "Warning: vglserver_config reported issues (see /tmp/vglserver_config.log)"
    fi
fi

# Launch a virtual X server for the VNC session
log "Starting Xvfb on display ${VNC_DISPLAY} (${VNC_RESOLUTION}x${VNC_DEPTH})"
Xvfb "$VNC_DISPLAY" -screen 0 "${VNC_RESOLUTION}x${VNC_DEPTH}" -nolisten tcp -noreset &
PIDS="${PIDS:-} $!"

# Wait for the display to become ready
for _ in $(seq 1 20); do
    if xdpyinfo -display "$VNC_DISPLAY" >/dev/null 2>&1; then
        break
    fi
    sleep 0.5
done

if ! xdpyinfo -display "$VNC_DISPLAY" >/dev/null 2>&1; then
    fatal "Virtual display ${VNC_DISPLAY} did not become available"
fi

# Lightweight window manager keeps editor happy
log "Starting fluxbox window manager"
DISPLAY="$VNC_DISPLAY" fluxbox >/tmp/fluxbox.log 2>&1 &
PIDS="$PIDS $!"

# Start VNC server that mirrors the virtual display
log "Starting x11vnc server on port ${VNC_PORT}"
X11VNC_FLAGS=(-display "$VNC_DISPLAY" -forever -shared -rfbport "$VNC_PORT" -noxdamage -repeat -quiet)
if [ -n "$VNC_PASSWORD" ]; then
    X11VNC_FLAGS+=(-passwd "$VNC_PASSWORD")
else
    X11VNC_FLAGS+=(-nopw)
fi
x11vnc "${X11VNC_FLAGS[@]}" &
PIDS="$PIDS $!"
log "x11vnc running - connect with your VNC client to localhost:${VNC_PORT}"

# Optional Asset Processor companion
if [ "$O3DE_ASSET_PROCESSOR" != "0" ] && [ -x "$ASSET_PROCESSOR_BIN" ]; then
    log "Starting Asset Processor in background"
    "$ASSET_PROCESSOR_BIN" --start-hidden --project-path "$PROJECT_ROOT" >/tmp/asset_processor.log 2>&1 &
    PIDS="$PIDS $!"
else
    log "Skipping Asset Processor launch (O3DE_ASSET_PROCESSOR=$O3DE_ASSET_PROCESSOR)"
fi

# Build argument array for O3DE executable
O3DE_ARGS_ARRAY=()
if [ -n "$O3DE_ARGS" ]; then
    # shellcheck disable=SC2206
    O3DE_ARGS_ARRAY=($O3DE_ARGS)
fi

export DISPLAY="$VNC_DISPLAY"
export VGL_DISPLAY="$VGL_DISPLAY"
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=xcb

log "Launching ${O3DE_EXECUTABLE} with VirtualGL (DISPLAY=$DISPLAY, VGL_DISPLAY=$VGL_DISPLAY)"
vglrun -d "$VGL_DISPLAY" "$TARGET_BIN" "${O3DE_ARGS_ARRAY[@]}"
