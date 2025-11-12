#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${REPO_ROOT}/.env"

LOG_PREFIX="[ros1_bridge_setup]"

log() {
    echo "${LOG_PREFIX} $*"
}

fatal() {
    echo "${LOG_PREFIX} ERROR: $*" >&2
    exit 1
}

if [[ -f "${ENV_FILE}" ]]; then
    # shellcheck disable=SC1090
    set -a
    source "${ENV_FILE}"
    set +a
fi

resolve_path() {
    local raw="$1"
    if [[ -z "${raw}" ]]; then
        fatal "resolve_path called with empty argument"
    fi
    if [[ "${raw}" == ~* ]]; then
        raw="${HOME}${raw:1}"
    fi
    if [[ "${raw}" = /* ]]; then
        printf '%s\n' "${raw}"
    else
        printf '%s\n' "${REPO_ROOT}/${raw#./}"
    fi
}

BRIDGE_PATH="$(resolve_path "${ROS1_BRIDGE_HOST_PATH:-./ros-humble-ros1-bridge}")"
BRIDGE_SENTINEL="${BRIDGE_PATH}/install/setup.bash"
BUILDER_REPO_URL="${ROS1_BRIDGE_BUILDER_REPO:-https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git}"
BUILDER_REF="${ROS1_BRIDGE_BUILDER_REF:-main}"
BUILDER_DIR="$(resolve_path "${ROS1_BRIDGE_BUILDER_DIR:-./.cache/ros-humble-ros1-bridge-builder}")"
BUILDER_IMAGE="${ROS1_BRIDGE_BUILDER_IMAGE:-ros-humble-ros1-bridge-builder}"

if [[ -f "${BRIDGE_SENTINEL}" ]]; then
    log "Bridge assets already present at ${BRIDGE_PATH}"
    exit 0
fi

command -v docker >/dev/null 2>&1 || fatal "docker is required but not found in PATH"
command -v git >/dev/null 2>&1 || fatal "git is required but not found in PATH"

mkdir -p "$(dirname "${BUILDER_DIR}")"

if [[ -d "${BUILDER_DIR}/.git" ]]; then
    log "Updating bridge builder sources in ${BUILDER_DIR}"
    git -C "${BUILDER_DIR}" fetch --tags --force origin
    git -C "${BUILDER_DIR}" checkout "${BUILDER_REF}"
    git -C "${BUILDER_DIR}" pull --ff-only origin "${BUILDER_REF}"
else
    log "Cloning bridge builder sources to ${BUILDER_DIR}"
    git clone --branch "${BUILDER_REF}" --depth 1 "${BUILDER_REPO_URL}" "${BUILDER_DIR}"
fi

log "Building ${BUILDER_IMAGE} image (this may take a minute)"
docker build -t "${BUILDER_IMAGE}" "${BUILDER_DIR}"

TEMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TEMP_DIR}"' EXIT

log "Extracting ROS1 bridge artifacts"
if ! docker run --rm "${BUILDER_IMAGE}" | tar -xz -C "${TEMP_DIR}" -f -; then
    fatal "Failed to extract bridge artifacts from builder image"
fi

if [[ ! -d "${TEMP_DIR}/ros-humble-ros1-bridge" ]]; then
    fatal "Bridge archive did not contain ros-humble-ros1-bridge directory"
fi

log "Installing bridge assets to ${BRIDGE_PATH}"
rm -rf "${BRIDGE_PATH}"
mkdir -p "$(dirname "${BRIDGE_PATH}")"
mv "${TEMP_DIR}/ros-humble-ros1-bridge" "${BRIDGE_PATH}"

log "Bridge assets ready (${BRIDGE_PATH})"
