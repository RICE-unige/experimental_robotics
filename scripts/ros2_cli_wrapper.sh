#!/bin/bash
set -euo pipefail

REAL_ROS2_BIN="/opt/ros/${ROS_DISTRO:-humble}/bin/ros2"
if [ ! -x "$REAL_ROS2_BIN" ]; then
    REAL_ROS2_BIN="$(command -v ros2.real 2>/dev/null || true)"
fi

if [ -z "${REAL_ROS2_BIN}" ] || [ ! -x "${REAL_ROS2_BIN}" ]; then
    echo "[ros2-wrapper] Unable to locate the upstream ros2 binary." >&2
    exit 127
fi

if [ $# -eq 0 ]; then
    exec "${REAL_ROS2_BIN}"
fi

ADD_SPIN=false
DEFAULT_SPIN="${ROS2CLI_SPIN_TIME:-5}"

if [ "${ROS2CLI_FORCE_NO_DAEMON:-}" = "1" ]; then
    SUBCOMMAND="${1:-}"
    VERB="${2:-}"
    case "${SUBCOMMAND}" in
        topic)
            case "${VERB}" in
                list|info|hz|bw|echo)
                    ADD_SPIN=true
                    ;;
            esac
            ;;
        node|service|action)
            if [ "${VERB}" = "list" ]; then
                ADD_SPIN=true
            fi
            ;;
    esac
fi

EXTRA_ARGS=()
if ${ADD_SPIN}; then
    for arg in "$@"; do
        case "${arg}" in
            --spin-time|--spin-time=*|-h|--help)
                ADD_SPIN=false
                break
                ;;
        esac
    done
    if ${ADD_SPIN}; then
        EXTRA_ARGS+=(--spin-time "${DEFAULT_SPIN}")
    fi
fi

exec "${REAL_ROS2_BIN}" "$@" "${EXTRA_ARGS[@]}"
