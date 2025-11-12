#!/bin/bash
set -eo pipefail

BRIDGE_PATH="${ROS1_BRIDGE_CONTAINER_PATH:-/root/ros-humble-ros1-bridge}"
ROS1_MASTER_URI="${ROS1_MASTER_URI:-}"
ROS1_LOCAL_IP="${ROS1_LOCAL_IP:-}"
TOPICS_FILE="${ROS1_BRIDGE_TOPICS_FILE:-/root/config/ros1_bridge/topics.yaml}"
unset ROS2CLI_DISABLE_DAEMON
unset ROS2CLI_FORCE_NO_DAEMON
unset ROS2CLI_SPIN_TIME

STRATEGY_FILE="/opt/ros/humble/lib/python3.10/site-packages/ros2cli/node/strategy.py"

log() {
    echo "[ros2_bridge] $*"
}

fatal() {
    echo "[ros2_bridge][ERROR] $*" >&2
    exit 1
}

if [ -z "$ROS1_MASTER_URI" ]; then
    fatal "ROS1_MASTER_URI is not set. Update your .env (e.g., http://<rosbot-ip>:11311)."
fi

if [ -z "$ROS1_LOCAL_IP" ]; then
    fatal "ROS1_LOCAL_IP is not set. It should be your laptop's IP on the robot network."
fi

if [ ! -d "$BRIDGE_PATH" ]; then
    fatal "Bridge directory '$BRIDGE_PATH' not found. Run ./scripts/setup_ros1_bridge.sh on the host (./run.sh real auto-runs it) or update ROS1_BRIDGE_HOST_PATH."
fi

if [ ! -f "$BRIDGE_PATH/install/local_setup.bash" ]; then
    fatal "Expected '$BRIDGE_PATH/install/local_setup.bash'. Re-run ./scripts/setup_ros1_bridge.sh to regenerate the assets."
fi

if [ ! -f "$TOPICS_FILE" ]; then
    fatal "Topics file '$TOPICS_FILE' not found. Provide a YAML file with the parameter_bridge configuration."
fi

ROS_MASTER_URI_VALUE="$ROS1_MASTER_URI"
ROS_IP_VALUE="$ROS1_LOCAL_IP"

export ROS_MASTER_URI="$ROS_MASTER_URI_VALUE"
export ROS_IP="$ROS_IP_VALUE"
export ROS_HOSTNAME="$ROS_IP_VALUE"

log "Sourcing ROS2 Humble..."
: "${AMENT_TRACE_SETUP_FILES:=}"
export AMENT_TRACE_SETUP_FILES
source /opt/ros/humble/setup.bash

log "Sourcing ros1_bridge from $BRIDGE_PATH..."
source "$BRIDGE_PATH/install/local_setup.bash"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

log "Bridge configuration:"
log "  ROS_MASTER_URI: $ROS_MASTER_URI_VALUE"
log "  ROS_IP / ROS_HOSTNAME: $ROS_IP_VALUE"
log "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
log "  Topics file: $TOPICS_FILE"

log "Loading parameter_bridge topics into ROS1 parameter server..."
if ! TOPICS_FILE_TARGET="$TOPICS_FILE" python3 - <<'PY'
import os
import sys
from xmlrpc.client import ServerProxy

try:
    import yaml
except ImportError as exc:
    print(f"[ros2_bridge][ERROR] Missing PyYAML dependency: {exc}", file=sys.stderr)
    sys.exit(1)

topics_file = os.environ["TOPICS_FILE_TARGET"]
master_uri = os.environ.get("ROS_MASTER_URI")
if not master_uri:
    print("[ros2_bridge][ERROR] ROS_MASTER_URI is not set for parameter upload", file=sys.stderr)
    sys.exit(1)

with open(topics_file, "r", encoding="utf-8") as fh:
    data = yaml.safe_load(fh) or {}

if "topics" not in data:
    print("[ros2_bridge][ERROR] Topics YAML must define a top-level 'topics' key", file=sys.stderr)
    sys.exit(1)

proxy = ServerProxy(master_uri)
caller = "/ros2_bridge_param_loader"

try:
    proxy.deleteParam(caller, "topics")
except Exception:
    pass

try:
    proxy.setParam(caller, "topics", data["topics"])
except Exception as exc:
    print(f"[ros2_bridge][ERROR] Failed to push topics parameter: {exc}", file=sys.stderr)
    sys.exit(1)
PY
then
    fatal "Failed to load parameter_bridge topics into ROS1 parameter server."
fi

log "Topics parameter loaded."

log "Starting ros1_bridge parameter_bridge..."
exec ros2 run ros1_bridge parameter_bridge
