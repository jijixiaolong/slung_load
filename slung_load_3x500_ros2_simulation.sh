#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSION="${SESSION:-px4_slung_load_3x500_ros2}"
PX4_DIR="${PX4_DIR:-$SCRIPT_DIR}"
AUTOSTART_ID="${AUTOSTART_ID:-4001}"
ATTACH_TMUX="${ATTACH_TMUX:-1}"
SKIP_BUILD="${SKIP_BUILD:-0}"
START_UXRCE_AGENT="${START_UXRCE_AGENT:-1}"
START_ROS2="${START_ROS2:-1}"
UAV2_DELAY_SEC="${UAV2_DELAY_SEC:-0}"
UAV3_DELAY_SEC="${UAV3_DELAY_SEC:-0}"
WAIT_WORLD_TIMEOUT_SEC="${WAIT_WORLD_TIMEOUT_SEC:-90}"
WAIT_SENSOR_TIMEOUT_SEC="${WAIT_SENSOR_TIMEOUT_SEC:-60}"
KILL_ALL_GZ_SIM="${KILL_ALL_GZ_SIM:-1}"
GZ_PARTITION_NAME="${GZ_PARTITION_NAME:-px4_slung_load_3x500}"

HEADLESS="${HEADLESS:-}"
GZ_VERBOSE="${GZ_VERBOSE:-0}"

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
ROS2_WS="${ROS2_WS:-}"
ROS2_CMD="${ROS2_CMD:-ros2 topic list}"
SIM_MODEL="${SIM_MODEL:-gz_x500}"
START_PAYLOAD_POSITION_BRIDGE="${START_PAYLOAD_POSITION_BRIDGE:-1}"
PAYLOAD_ENTITY_NAMES="${PAYLOAD_ENTITY_NAMES:-slung_load::payload_link,payload_link,slung_load}"
PAYLOAD_POSITION_TOPIC="${PAYLOAD_POSITION_TOPIC:-/payload/position}"
PAYLOAD_BRIDGE_FRAME_ID="${PAYLOAD_BRIDGE_FRAME_ID:-world}"
PAYLOAD_NAVSAT_TOPIC="${PAYLOAD_NAVSAT_TOPIC:-/payload/navsat}"
PAYLOAD_NAVSAT_FRAME_ID="${PAYLOAD_NAVSAT_FRAME_ID:-payload_gps}"
PAYLOAD_PX4_GPS_TOPIC="${PAYLOAD_PX4_GPS_TOPIC:-/payload/vehicle_gps_position}"
PAYLOAD_PX4_GPS_DEVICE_ID="${PAYLOAD_PX4_GPS_DEVICE_ID:-0}"
PAYLOAD_GPS_SATELLITES_USED="${PAYLOAD_GPS_SATELLITES_USED:-10}"
PAYLOAD_BRIDGE_FILE="${PAYLOAD_BRIDGE_FILE:-}"
PAYLOAD_BRIDGE_LOG="${PAYLOAD_BRIDGE_LOG:-/tmp/px4_slung_load_payload_position_bridge.log}"

WORLD_SOURCE_DIR="${WORLD_SOURCE_DIR:-$PX4_DIR/Tools/simulation/gz/slung_load/worlds}"
WORLD_INSTALL_DIR="${WORLD_INSTALL_DIR:-$PX4_DIR/Tools/simulation/gz/worlds}"

BUILD_DIR=""
PX4_BIN=""
PX4_ETC_DIR=""
PX4_ROOTFS_DIR=""
WORLD_NAME="${WORLD_NAME:-}"
WORLD_SDF_SRC=""
WORLD_SDF_DST=""

usage() {
    cat <<'EOF'
Usage:
  ./slung_load_3x500_ros2_simulation.sh [standard|0.8m|large_2m|2m|WORLD_NAME] [--dry-run]

Examples:
  ./slung_load_3x500_ros2_simulation.sh
  ./slung_load_3x500_ros2_simulation.sh 2m
  WORLD_NAME=multi_uav_slung_load_r1p15 ./slung_load_3x500_ros2_simulation.sh
  ROS2_WS=/home/ubuntu22/swarm_ws ./slung_load_3x500_ros2_simulation.sh 2m
EOF
}

require_cmd() {
    local cmd="$1"
    local hint="$2"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "$cmd not found. $hint"
        exit 1
    fi
}

tmux_send_cmd() {
    local pane="$1"
    local cmd="$2"
    tmux send-keys -t "$pane" "$cmd" C-m
}

stop_matching_processes() {
    local pattern="$1"

    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1

    if pgrep -f "$pattern" >/dev/null 2>&1; then
        pkill -KILL -f "$pattern" 2>/dev/null || true
    fi
}

has_topic() {
    local topics="$1"
    local topic="$2"
    printf '%s\n' "$topics" | grep -Fqx "$topic"
}

wait_for_world_ready() {
    local timeout_sec="$1"

    if ! command -v gz >/dev/null 2>&1; then
        echo "gz not found. Skip explicit world readiness check."
        return 1
    fi

    echo "Waiting for Gazebo world /world/$WORLD_NAME/scene/info (${timeout_sec}s timeout)"
    for _ in $(seq 1 "$timeout_sec"); do
        if gz service -i --service "/world/$WORLD_NAME/scene/info" >/dev/null 2>&1; then
            echo "Gazebo world is ready."
            return 0
        fi
        sleep 1
    done

    return 1
}

wait_for_model_sensor_topics() {
    local model_name="$1"
    local timeout_sec="$2"

    if ! command -v gz >/dev/null 2>&1; then
        echo "gz not found. Skip model sensor topic check for $model_name."
        return 1
    fi

    local base_topic="/world/$WORLD_NAME/model/$model_name/link/base_link/sensor"

    echo "Waiting model sensor topics for $model_name (${timeout_sec}s timeout)"
    for _ in $(seq 1 "$timeout_sec"); do
        local topics
        topics="$(gz topic -l 2>/dev/null || true)"

        if has_topic "$topics" "$base_topic/imu_sensor/imu" && \
           has_topic "$topics" "$base_topic/air_pressure_sensor/air_pressure" && \
           has_topic "$topics" "$base_topic/magnetometer_sensor/magnetometer" && \
           has_topic "$topics" "$base_topic/navsat_sensor/navsat"; then
            echo "Model sensor topics ready: $model_name"
            return 0
        fi

        sleep 1
    done

    return 1
}

wait_for_px4_sensor_ready() {
    local instance="$1"
    local timeout_sec="$2"
    local listener_bin="$BUILD_DIR/bin/px4-listener"

    if [ ! -x "$listener_bin" ]; then
        echo "px4-listener not found, skip PX4 sensor readiness check."
        return 1
    fi

    if ! command -v timeout >/dev/null 2>&1; then
        echo "timeout command not found, skip PX4 sensor readiness check."
        return 1
    fi

    echo "Waiting PX4 instance $instance sensor_combined (${timeout_sec}s timeout)"
    for _ in $(seq 1 "$timeout_sec"); do
        if timeout 2s "$listener_bin" --instance "$instance" sensor_combined -n 1 >/dev/null 2>&1; then
            echo "PX4 instance $instance sensor_combined is ready."
            return 0
        fi
        sleep 1
    done

    return 1
}

setup_gz_env() {
    export PX4_GZ_MODELS="${PX4_GZ_MODELS:-$PX4_DIR/Tools/simulation/gz/models}"
    export PX4_GZ_WORLDS="${PX4_GZ_WORLDS:-$WORLD_INSTALL_DIR}"
    export PX4_GZ_PLUGINS="${PX4_GZ_PLUGINS:-$BUILD_DIR/src/modules/simulation/gz_plugins}"
    export PX4_GZ_SERVER_CONFIG="${PX4_GZ_SERVER_CONFIG:-$PX4_DIR/src/modules/simulation/gz_bridge/server.config}"

    local gz_resource_prefix="${GZ_SIM_RESOURCE_PATH:-}"
    local gz_plugin_prefix="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

    if [ -n "$gz_resource_prefix" ]; then
        gz_resource_prefix="${gz_resource_prefix}:"
    fi
    if [ -n "$gz_plugin_prefix" ]; then
        gz_plugin_prefix="${gz_plugin_prefix}:"
    fi

    export GZ_SIM_RESOURCE_PATH="${gz_resource_prefix}${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="${gz_plugin_prefix}${PX4_GZ_PLUGINS}"
    export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"
    export GZ_PARTITION="${GZ_PARTITION_NAME}"

    echo "Gazebo env:"
    echo "  PX4_GZ_MODELS=$PX4_GZ_MODELS"
    echo "  PX4_GZ_WORLDS=$PX4_GZ_WORLDS"
    echo "  PX4_GZ_PLUGINS=$PX4_GZ_PLUGINS"
    echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
    echo "  GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"
    echo "  GZ_SIM_SERVER_CONFIG_PATH=$GZ_SIM_SERVER_CONFIG_PATH"
    echo "  GZ_PARTITION=$GZ_PARTITION"
}

sync_tmux_env() {
    tmux set-environment -t "$SESSION" PX4_GZ_MODELS "$PX4_GZ_MODELS"
    tmux set-environment -t "$SESSION" PX4_GZ_WORLDS "$PX4_GZ_WORLDS"
    tmux set-environment -t "$SESSION" PX4_GZ_PLUGINS "$PX4_GZ_PLUGINS"
    tmux set-environment -t "$SESSION" PX4_GZ_SERVER_CONFIG "$PX4_GZ_SERVER_CONFIG"
    tmux set-environment -t "$SESSION" GZ_SIM_RESOURCE_PATH "$GZ_SIM_RESOURCE_PATH"
    tmux set-environment -t "$SESSION" GZ_SIM_SYSTEM_PLUGIN_PATH "$GZ_SIM_SYSTEM_PLUGIN_PATH"
    tmux set-environment -t "$SESSION" GZ_SIM_SERVER_CONFIG_PATH "$GZ_SIM_SERVER_CONFIG_PATH"
    tmux set-environment -t "$SESSION" GZ_PARTITION "$GZ_PARTITION"
}

prepare_px4_runtime() {
    if [ ! -d "$PX4_ROOTFS_DIR" ]; then
        echo "Create missing rootfs: $PX4_ROOTFS_DIR"
        mkdir -p "$PX4_ROOTFS_DIR"
    fi
}

refresh_existing_px4_build() {
    local ninja_file="$BUILD_DIR/build.ninja"
    local cache_file="$BUILD_DIR/CMakeCache.txt"

    if [ ! -f "$ninja_file" ] || [ ! -f "$cache_file" ]; then
        return
    fi

    if ! command -v cmake >/dev/null 2>&1; then
        echo "cmake not found, skip build tree refresh."
        return
    fi

    # Refresh imported system package targets after host package upgrades.
    echo "Refresh existing CMake build tree: $BUILD_DIR"
    cmake -S "$PX4_DIR" -B "$BUILD_DIR"
}

install_world_file() {
    if [ ! -f "$WORLD_SDF_SRC" ]; then
        echo "World file missing: $WORLD_SDF_SRC"
        echo "Generate it first, for example:"
        echo "  cd $PX4_DIR/Tools/simulation/gz/slung_load"
        echo "  python3 gen_sdf.py --radius 0.8 --out worlds/multi_uav_slung_load_r0p80.sdf"
        echo "  python3 gen_sdf.py --radius 1.1547 --out worlds/multi_uav_slung_load_r1p15.sdf"
        exit 1
    fi

    mkdir -p "$WORLD_INSTALL_DIR"
    if [ ! -f "$WORLD_SDF_DST" ] || [ "$WORLD_SDF_SRC" -nt "$WORLD_SDF_DST" ]; then
        echo "Install world file: $WORLD_SDF_SRC -> $WORLD_SDF_DST"
        cp "$WORLD_SDF_SRC" "$WORLD_SDF_DST"
    fi
}

write_payload_position_bridge() {
    if [ "$START_PAYLOAD_POSITION_BRIDGE" != "1" ]; then
        return
    fi

    if [ -z "$PAYLOAD_BRIDGE_FILE" ]; then
        PAYLOAD_BRIDGE_FILE="$PX4_ROOTFS_DIR/slung_load_payload_position_bridge.py"
    fi

    mkdir -p "$(dirname "$PAYLOAD_BRIDGE_FILE")"

    cat > "$PAYLOAD_BRIDGE_FILE" <<'PYEOF'
#!/usr/bin/env python3

import argparse
import json
import math
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6378137.0
DEFAULT_REF_LAT_DEG = 47.397742
DEFAULT_REF_LON_DEG = 8.545594
DEFAULT_REF_ALT_M = 488.0
GPS_EPH_M = 0.9
GPS_EPV_M = 1.78
GPS_SPEED_VARIANCE_M_S = 0.4
GPS_COURSE_VARIANCE_RAD = 0.1
GPS_HDOP = 0.7
GPS_VDOP = 1.1


def _safe_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _parse_entities(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def _safe_text(value: Optional[str]) -> str:
    return value.strip() if isinstance(value, str) else ""


def _matches_entity(name: str, entities: list[str]) -> bool:
    if name in entities:
        return True

    for entity in entities:
        if "::" not in entity and name.endswith(f"::{entity}"):
            return True

    return False


def _load_world_geo_reference(world_sdf: str) -> tuple[float, float, float, float, str]:
    default_reference = (
        DEFAULT_REF_LAT_DEG,
        DEFAULT_REF_LON_DEG,
        DEFAULT_REF_ALT_M,
        0.0,
        "ENU",
    )

    world_sdf = _safe_text(world_sdf)
    if not world_sdf:
        return default_reference

    try:
        root = ET.parse(world_sdf).getroot()
    except (ET.ParseError, OSError):
        return default_reference

    spherical = root.find(".//world/spherical_coordinates")
    if spherical is None:
        spherical = root.find(".//spherical_coordinates")

    if spherical is None:
        return default_reference

    return (
        _safe_float(spherical.findtext("latitude_deg"), DEFAULT_REF_LAT_DEG),
        _safe_float(spherical.findtext("longitude_deg"), DEFAULT_REF_LON_DEG),
        _safe_float(spherical.findtext("elevation"), DEFAULT_REF_ALT_M),
        _safe_float(spherical.findtext("heading_deg"), 0.0),
        (_safe_text(spherical.findtext("world_frame_orientation")) or "ENU").upper(),
    )


def _rotate_world_xy(x: float, y: float, heading_deg: float) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    east = x * math.cos(heading_rad) - y * math.sin(heading_rad)
    north = x * math.sin(heading_rad) + y * math.cos(heading_rad)
    return east, north


def _world_to_navsat(
    x: float,
    y: float,
    z: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    heading_deg: float,
) -> tuple[float, float, float]:
    east_m, north_m = _rotate_world_xy(x, y, heading_deg)
    latitude_deg = ref_lat_deg + math.degrees(north_m / EARTH_RADIUS_M)
    cos_lat = max(abs(math.cos(math.radians(ref_lat_deg))), 1e-6)
    longitude_deg = ref_lon_deg + math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))
    altitude_m = ref_alt_m + z
    return latitude_deg, longitude_deg, altitude_m


def _stamp_fields(message: dict, node) -> tuple[int, int, int]:
    stamp = message.get("header", {}).get("stamp", {})
    sec = _safe_int(stamp.get("sec"), 0) if isinstance(stamp, dict) else 0
    nanosec = _safe_int(stamp.get("nsec"), 0) if isinstance(stamp, dict) else 0

    if sec == 0 and nanosec == 0:
        now_ns = node.get_clock().now().nanoseconds
        sec = now_ns // 1_000_000_000
        nanosec = now_ns % 1_000_000_000

    timestamp_us = sec * 1_000_000 + nanosec // 1_000
    return sec, nanosec, timestamp_us


def _load_px4_sensor_gps(node):
    try:
        from px4_msgs.msg import SensorGps as Px4SensorGps
    except ImportError:
        node.get_logger().warning(
            "px4_msgs.msg.SensorGps not available, skip PX4-style payload GPS topic"
        )
        return None

    return Px4SensorGps


def _set_if_present(msg: object, field: str, value: object) -> None:
    if hasattr(msg, field):
        setattr(msg, field, value)


def _build_px4_sensor_gps(
    msg_cls,
    timestamp_us: int,
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    vel_north_m_s: float,
    vel_east_m_s: float,
    vel_down_m_s: float,
    device_id: int,
    satellites_used: int,
) -> object:
    msg = msg_cls()
    speed_m_s = math.hypot(vel_north_m_s, vel_east_m_s)

    _set_if_present(msg, "timestamp", timestamp_us)
    _set_if_present(msg, "timestamp_sample", timestamp_us)
    _set_if_present(msg, "device_id", device_id)
    _set_if_present(msg, "latitude_deg", latitude_deg)
    _set_if_present(msg, "longitude_deg", longitude_deg)
    _set_if_present(msg, "altitude_msl_m", altitude_m)
    _set_if_present(msg, "altitude_ellipsoid_m", altitude_m)
    _set_if_present(msg, "s_variance_m_s", GPS_SPEED_VARIANCE_M_S)
    _set_if_present(msg, "c_variance_rad", GPS_COURSE_VARIANCE_RAD)
    _set_if_present(msg, "fix_type", getattr(msg_cls, "FIX_TYPE_3D", 3))
    _set_if_present(msg, "eph", GPS_EPH_M)
    _set_if_present(msg, "epv", GPS_EPV_M)
    _set_if_present(msg, "hdop", GPS_HDOP)
    _set_if_present(msg, "vdop", GPS_VDOP)
    _set_if_present(msg, "noise_per_ms", 0)
    _set_if_present(msg, "automatic_gain_control", 0)
    _set_if_present(msg, "jamming_state", getattr(msg_cls, "JAMMING_STATE_UNKNOWN", 0))
    _set_if_present(msg, "jamming_indicator", 0)
    _set_if_present(msg, "spoofing_state", getattr(msg_cls, "SPOOFING_STATE_UNKNOWN", 0))
    _set_if_present(msg, "vel_m_s", speed_m_s)
    _set_if_present(msg, "vel_n_m_s", vel_north_m_s)
    _set_if_present(msg, "vel_e_m_s", vel_east_m_s)
    _set_if_present(msg, "vel_d_m_s", vel_down_m_s)
    _set_if_present(msg, "cog_rad", math.atan2(vel_east_m_s, vel_north_m_s))
    _set_if_present(msg, "vel_ned_valid", True)
    _set_if_present(msg, "timestamp_time_relative", 0)
    _set_if_present(msg, "time_utc_usec", 0)
    _set_if_present(msg, "satellites_used", satellites_used)
    _set_if_present(msg, "heading", float("nan"))
    _set_if_present(msg, "heading_offset", float("nan"))
    _set_if_present(msg, "heading_accuracy", 0.0)
    _set_if_present(msg, "rtcm_injection_rate", 0.0)
    _set_if_present(msg, "selected_rtcm_instance", 0)
    _set_if_present(msg, "rtcm_crc_failed", False)
    _set_if_present(msg, "rtcm_msg_used", getattr(msg_cls, "RTCM_MSG_USED_NOT_USED", 1))

    return msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--world-sdf", default="")
    parser.add_argument("--entities", default="slung_load::payload_link,payload_link,slung_load")
    parser.add_argument("--topic", "--position-topic", dest="position_topic", default="/payload/position")
    parser.add_argument("--frame-id", "--position-frame-id", dest="position_frame_id", default="world")
    parser.add_argument("--navsat-topic", default="/payload/navsat")
    parser.add_argument("--navsat-frame-id", default="payload_gps")
    parser.add_argument("--px4-gps-topic", default="/payload/vehicle_gps_position")
    parser.add_argument("--px4-device-id", type=int, default=0)
    parser.add_argument("--satellites-used", type=int, default=10)
    args = parser.parse_args()

    entities = _parse_entities(args.entities)
    if not entities:
        print("No payload entities configured", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node = rclpy.create_node("payload_state_bridge")

    position_topic = _safe_text(args.position_topic)
    navsat_topic = _safe_text(args.navsat_topic)
    px4_gps_topic = _safe_text(args.px4_gps_topic)

    position_publisher = None
    navsat_publisher = None
    px4_gps_publisher = None
    px4_gps_msg_cls = None

    if position_topic:
        position_publisher = node.create_publisher(PointStamped, position_topic, 10)

    if navsat_topic:
        navsat_publisher = node.create_publisher(NavSatFix, navsat_topic, 10)

    if px4_gps_topic:
        px4_gps_msg_cls = _load_px4_sensor_gps(node)
        if px4_gps_msg_cls is not None:
            px4_gps_publisher = node.create_publisher(px4_gps_msg_cls, px4_gps_topic, 10)

    if position_publisher is None and navsat_publisher is None and px4_gps_publisher is None:
        node.get_logger().error("No payload outputs are enabled")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ref_lat_deg, ref_lon_deg, ref_alt_m, ref_heading_deg, frame_orientation = _load_world_geo_reference(
        args.world_sdf
    )
    if frame_orientation != "ENU":
        node.get_logger().warning(
            f"Unsupported world_frame_orientation={frame_orientation}, assume ENU"
        )

    gz_topic = f"/world/{args.world}/dynamic_pose/info"
    process: Optional[subprocess.Popen[str]] = None
    running = True
    previous_pose: Optional[tuple[float, float, float, int]] = None

    def _handle_signal(signum, _frame):
        nonlocal running
        running = False
        node.get_logger().info(f"Received signal {signum}, stopping bridge")

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", gz_topic, "--json-output"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        node.get_logger().error("gz command not found")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info(
        "Bridge started: "
        f"{gz_topic} -> position={position_topic or '<disabled>'}, "
        f"navsat={navsat_topic or '<disabled>'}, "
        f"px4_gps={px4_gps_topic or '<disabled>'} "
        f"(entities={','.join(entities)})"
    )
    node.get_logger().info(
        "GPS reference: "
        f"lat={ref_lat_deg:.7f}, lon={ref_lon_deg:.7f}, alt={ref_alt_m:.3f}, "
        f"heading={ref_heading_deg:.2f}, frame={frame_orientation}"
    )

    try:
        while running and rclpy.ok():
            if process.stdout is None:
                break

            line = process.stdout.readline()
            if line == "":
                if process.poll() is not None:
                    break
                rclpy.spin_once(node, timeout_sec=0.01)
                continue

            line = line.strip()
            if not line:
                continue

            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue

            pose_list = message.get("pose")
            if not isinstance(pose_list, list):
                continue

            target = next(
                (
                    item
                    for item in pose_list
                    if isinstance(item, dict)
                    and _matches_entity(str(item.get("name", "")), entities)
                ),
                None,
            )

            if target is None:
                continue

            position = target.get("position", {})
            if not isinstance(position, dict):
                continue

            x = _safe_float(position.get("x"), 0.0)
            y = _safe_float(position.get("y"), 0.0)
            z = _safe_float(position.get("z"), 0.0)
            stamp_sec, stamp_nanosec, timestamp_us = _stamp_fields(message, node)

            vel_north_m_s = 0.0
            vel_east_m_s = 0.0
            vel_down_m_s = 0.0

            if previous_pose is not None:
                prev_x, prev_y, prev_z, prev_timestamp_us = previous_pose
                dt_us = timestamp_us - prev_timestamp_us
                if dt_us > 0:
                    dt_s = dt_us / 1_000_000.0
                    vel_east_m_s = (x - prev_x) / dt_s
                    vel_north_m_s = (y - prev_y) / dt_s
                    vel_down_m_s = -(z - prev_z) / dt_s

            previous_pose = (x, y, z, timestamp_us)

            latitude_deg, longitude_deg, altitude_m = _world_to_navsat(
                x,
                y,
                z,
                ref_lat_deg,
                ref_lon_deg,
                ref_alt_m,
                ref_heading_deg,
            )

            if position_publisher is not None:
                position_msg = PointStamped()
                position_msg.header.frame_id = args.position_frame_id
                position_msg.header.stamp.sec = stamp_sec
                position_msg.header.stamp.nanosec = stamp_nanosec
                position_msg.point.x = x
                position_msg.point.y = y
                position_msg.point.z = z
                position_publisher.publish(position_msg)

            if navsat_publisher is not None:
                navsat_msg = NavSatFix()
                navsat_msg.header.frame_id = args.navsat_frame_id
                navsat_msg.header.stamp.sec = stamp_sec
                navsat_msg.header.stamp.nanosec = stamp_nanosec
                navsat_msg.status.status = NavSatStatus.STATUS_FIX
                navsat_msg.status.service = NavSatStatus.SERVICE_GPS
                navsat_msg.latitude = latitude_deg
                navsat_msg.longitude = longitude_deg
                navsat_msg.altitude = altitude_m
                navsat_msg.position_covariance = [
                    GPS_EPH_M * GPS_EPH_M, 0.0, 0.0,
                    0.0, GPS_EPH_M * GPS_EPH_M, 0.0,
                    0.0, 0.0, GPS_EPV_M * GPS_EPV_M,
                ]
                navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                navsat_publisher.publish(navsat_msg)

            if px4_gps_publisher is not None and px4_gps_msg_cls is not None:
                px4_gps_msg = _build_px4_sensor_gps(
                    px4_gps_msg_cls,
                    timestamp_us,
                    latitude_deg,
                    longitude_deg,
                    altitude_m,
                    vel_north_m_s,
                    vel_east_m_s,
                    vel_down_m_s,
                    args.px4_device_id,
                    max(args.satellites_used, 0),
                )
                px4_gps_publisher.publish(px4_gps_msg)

            rclpy.spin_once(node, timeout_sec=0.0)

    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
PYEOF

    chmod +x "$PAYLOAD_BRIDGE_FILE"
    echo "Payload position bridge file: $PAYLOAD_BRIDGE_FILE"
}

build_px4_cmd() {
    local instance="$1"
    local standalone="$2"
    local model_name="$3"

    local cmd=""
    if [ -n "$HEADLESS" ]; then
        cmd+="HEADLESS=$HEADLESS "
    fi
    cmd+="PX4_GZ_MODELS=\"$PX4_GZ_MODELS\" "
    cmd+="PX4_GZ_WORLDS=\"$PX4_GZ_WORLDS\" "
    cmd+="PX4_GZ_PLUGINS=\"$PX4_GZ_PLUGINS\" "
    cmd+="PX4_GZ_SERVER_CONFIG=\"$PX4_GZ_SERVER_CONFIG\" "
    cmd+="GZ_SIM_RESOURCE_PATH=\"$GZ_SIM_RESOURCE_PATH\" "
    cmd+="GZ_SIM_SYSTEM_PLUGIN_PATH=\"$GZ_SIM_SYSTEM_PLUGIN_PATH\" "
    cmd+="GZ_SIM_SERVER_CONFIG_PATH=\"$GZ_SIM_SERVER_CONFIG_PATH\" "
    cmd+="GZ_PARTITION=\"$GZ_PARTITION\" "
    cmd+="GZ_VERBOSE=$GZ_VERBOSE "
    if [ "$standalone" = "1" ]; then
        cmd+="PX4_GZ_STANDALONE=1 "
    fi
    cmd+="PX4_SYS_AUTOSTART=$AUTOSTART_ID "
    cmd+="PX4_SIM_MODEL=$SIM_MODEL "
    cmd+="PX4_GZ_WORLD=$WORLD_NAME "
    cmd+="PX4_GZ_MODEL_NAME=$model_name "
    cmd+="\"$PX4_BIN\" -i $instance -d \"$PX4_ETC_DIR\""

    printf '%s' "$cmd"
}

build_ros2_cmd() {
    local cmd=""

    cmd+="if [ ! -f \"$ROS2_SETUP\" ]; then echo '[ROS2] setup not found: $ROS2_SETUP'; exec bash; fi; "
    cmd+="source \"$ROS2_SETUP\"; "

    if [ -n "$ROS2_WS" ]; then
        cmd+="if [ -f \"$ROS2_WS/install/setup.bash\" ]; then source \"$ROS2_WS/install/setup.bash\"; else echo '[ROS2] workspace setup not found: $ROS2_WS/install/setup.bash'; fi; "
    fi

    cmd+="ros2 daemon start >/dev/null 2>&1 || true; "
    cmd+="echo '[ROS2] environment ready'; "

    if [ "$START_PAYLOAD_POSITION_BRIDGE" = "1" ]; then
        cmd+="if [ ! -f \"$PAYLOAD_BRIDGE_FILE\" ]; then "
        cmd+="echo '[ROS2] payload bridge file missing: $PAYLOAD_BRIDGE_FILE'; "
        cmd+="elif ! command -v python3 >/dev/null 2>&1; then "
        cmd+="echo '[ROS2] python3 not found, skip payload bridge'; "
        cmd+="elif ! command -v gz >/dev/null 2>&1; then "
        cmd+="echo '[ROS2] gz not found, skip payload bridge'; "
        cmd+="else "
        cmd+="echo '[ROS2] start payload bridge: position=$PAYLOAD_POSITION_TOPIC navsat=$PAYLOAD_NAVSAT_TOPIC px4_gps=$PAYLOAD_PX4_GPS_TOPIC'; "
        cmd+="python3 \"$PAYLOAD_BRIDGE_FILE\" --world \"$WORLD_NAME\" --world-sdf \"$WORLD_SDF_DST\" --entities \"$PAYLOAD_ENTITY_NAMES\" --position-topic \"$PAYLOAD_POSITION_TOPIC\" --position-frame-id \"$PAYLOAD_BRIDGE_FRAME_ID\" --navsat-topic \"$PAYLOAD_NAVSAT_TOPIC\" --navsat-frame-id \"$PAYLOAD_NAVSAT_FRAME_ID\" --px4-gps-topic \"$PAYLOAD_PX4_GPS_TOPIC\" --px4-device-id \"$PAYLOAD_PX4_GPS_DEVICE_ID\" --satellites-used \"$PAYLOAD_GPS_SATELLITES_USED\" >\"$PAYLOAD_BRIDGE_LOG\" 2>&1 & "
        cmd+="echo '[ROS2] payload bridge log: $PAYLOAD_BRIDGE_LOG'; "
        cmd+="fi; "
    fi

    cmd+="$ROS2_CMD; "
    cmd+="echo '[ROS2] command exited, keeping shell'; exec bash"

    printf '%s' "$cmd"
}

resolve_world_name() {
    local world_key="$1"

    if [ -n "${WORLD_NAME:-}" ]; then
        printf '%s' "$WORLD_NAME"
        return
    fi

    case "$world_key" in
        ""|2m) printf '%s' "multi_uav_slung_load_2m" ;;
        standard|0.8m) printf '%s' "multi_uav_slung_load_r0p80" ;;
        large_2m) printf '%s' "multi_uav_slung_load_r1p15" ;;
        *) printf '%s' "$world_key" ;;
    esac
}

WORLD_KEY=""
DRY_RUN=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --dry-run)
            DRY_RUN=1
            ;;
        standard|0.8m|large_2m|2m)
            if [ -n "$WORLD_KEY" ]; then
                echo "Only one world selector is allowed."
                exit 1
            fi
            WORLD_KEY="$1"
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            if [ -n "$WORLD_KEY" ]; then
                echo "Only one world selector is allowed."
                exit 1
            fi
            WORLD_KEY="$1"
            ;;
    esac
    shift
done

require_cmd tmux "Install with: sudo apt install tmux"

if [ ! -d "$PX4_DIR" ]; then
    echo "PX4_DIR does not exist: $PX4_DIR"
    exit 1
fi

cd "$PX4_DIR"

BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$BUILD_DIR/bin/px4"
PX4_ETC_DIR="$BUILD_DIR/etc"
PX4_ROOTFS_DIR="$BUILD_DIR/rootfs"
WORLD_NAME="$(resolve_world_name "$WORLD_KEY")"
WORLD_SDF_SRC="$WORLD_SOURCE_DIR/${WORLD_NAME}.sdf"
WORLD_SDF_DST="$WORLD_INSTALL_DIR/${WORLD_NAME}.sdf"

if [ "$DRY_RUN" = "1" ]; then
    echo "Session            : $SESSION"
    echo "World name         : $WORLD_NAME"
    echo "World source       : $WORLD_SDF_SRC"
    echo "World install      : $WORLD_SDF_DST"
    echo "PX4 binary         : $PX4_BIN"
    echo "PX4 etc            : $PX4_ETC_DIR"
    echo "PX4 rootfs         : $PX4_ROOTFS_DIR"
    echo "ROS2 setup         : $ROS2_SETUP"
    echo "ROS2 workspace     : ${ROS2_WS:-<unset>}"
    echo "Payload entities   : $PAYLOAD_ENTITY_NAMES"
    echo "Payload pos topic  : $PAYLOAD_POSITION_TOPIC"
    echo "Payload navsat     : $PAYLOAD_NAVSAT_TOPIC"
    echo "Payload px4 gps    : $PAYLOAD_PX4_GPS_TOPIC"
    [ -f "$WORLD_SDF_SRC" ] && echo "World source check : OK" || echo "World source check : MISSING"
    [ -x "$PX4_BIN" ] && echo "PX4 binary check   : OK" || echo "PX4 binary check   : MISSING"
    exit 0
fi

echo "[1/5] Build"
if [ "$SKIP_BUILD" = "1" ]; then
    echo "SKIP_BUILD=1, skip build"
else
    refresh_existing_px4_build
    make px4_sitl_default
fi

if [ ! -x "$PX4_BIN" ]; then
    echo "px4 binary missing: $PX4_BIN"
    exit 1
fi

if [ ! -d "$PX4_ETC_DIR" ]; then
    echo "px4 etc dir missing: $PX4_ETC_DIR"
    exit 1
fi

echo "[2/5] Prepare world and clean old processes"
install_world_file
tmux kill-session -t "$SESSION" 2>/dev/null || true
stop_matching_processes "$PX4_BIN -i"
if [ "$KILL_ALL_GZ_SIM" = "1" ]; then
    stop_matching_processes "gz sim"
else
    stop_matching_processes "gz sim.*${WORLD_NAME}[.]sdf"
fi
stop_matching_processes "MicroXRCEAgent udp4 -p 8888"
sleep 1

prepare_px4_runtime
setup_gz_env
write_payload_position_bridge

echo "[3/5] Start tmux session: $SESSION"
tmux new-session -d -s "$SESSION"
tmux rename-window -t "$SESSION":0 'SlungLoad3x500+ROS2'
tmux set-option -t "$SESSION" mouse on
sync_tmux_env

PANE_AGENT="$(tmux display-message -p -t "$SESSION":0.0 '#{pane_id}')"
PANE_PX4_1="$(tmux split-window -h -t "$PANE_AGENT" -P -F '#{pane_id}')"
PANE_PX4_2="$(tmux split-window -v -t "$PANE_AGENT" -P -F '#{pane_id}')"
PANE_PX4_3="$(tmux split-window -v -t "$PANE_PX4_1" -P -F '#{pane_id}')"
PANE_ROS2="$(tmux split-window -v -t "$PANE_PX4_2" -P -F '#{pane_id}')"

echo "[4/5] Launch processes"
if [ "$START_UXRCE_AGENT" = "1" ] && command -v MicroXRCEAgent >/dev/null 2>&1; then
    tmux_send_cmd "$PANE_AGENT" "MicroXRCEAgent udp4 -p 8888"
else
    tmux_send_cmd "$PANE_AGENT" "echo 'MicroXRCEAgent skipped (START_UXRCE_AGENT=0 or not installed)'"
fi

tmux_send_cmd "$PANE_PX4_1" "cd \"$PX4_DIR\""
tmux_send_cmd "$PANE_PX4_1" "$(build_px4_cmd 1 0 x500_0)"

if ! wait_for_world_ready "$WAIT_WORLD_TIMEOUT_SEC"; then
    echo "WARNING: world readiness check timed out; continue launching UAV2/UAV3"
fi

if ! wait_for_model_sensor_topics "x500_0" "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: x500_0 model sensor topics not ready in time."
fi

if ! wait_for_px4_sensor_ready 1 "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: PX4 instance 1 sensor_combined not ready in time."
fi

tmux_send_cmd "$PANE_PX4_2" "cd \"$PX4_DIR\""
if [ "$UAV2_DELAY_SEC" != "0" ]; then
    tmux_send_cmd "$PANE_PX4_2" "echo 'Delay UAV2 start: ${UAV2_DELAY_SEC}s'; sleep ${UAV2_DELAY_SEC}"
fi
tmux_send_cmd "$PANE_PX4_2" "$(build_px4_cmd 2 1 x500_1)"

if ! wait_for_model_sensor_topics "x500_1" "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: x500_1 model sensor topics not ready in time."
fi

if ! wait_for_px4_sensor_ready 2 "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: PX4 instance 2 sensor_combined not ready in time."
fi

tmux_send_cmd "$PANE_PX4_3" "cd \"$PX4_DIR\""
if [ "$UAV3_DELAY_SEC" != "0" ]; then
    tmux_send_cmd "$PANE_PX4_3" "echo 'Delay UAV3 start: ${UAV3_DELAY_SEC}s'; sleep ${UAV3_DELAY_SEC}"
fi
tmux_send_cmd "$PANE_PX4_3" "$(build_px4_cmd 3 1 x500_2)"

if ! wait_for_model_sensor_topics "x500_2" "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: x500_2 model sensor topics not ready in time."
fi

if ! wait_for_px4_sensor_ready 3 "$WAIT_SENSOR_TIMEOUT_SEC"; then
    echo "WARNING: PX4 instance 3 sensor_combined not ready in time."
fi

if [ "$START_ROS2" = "1" ]; then
    if command -v ros2 >/dev/null 2>&1; then
        tmux_send_cmd "$PANE_ROS2" "$(build_ros2_cmd)"
    else
        tmux_send_cmd "$PANE_ROS2" "echo 'ros2 not found in PATH; install ROS2 first'; exec bash"
    fi
else
    tmux_send_cmd "$PANE_ROS2" "echo 'ROS2 pane idle (START_ROS2=0)'; exec bash"
fi

tmux select-layout -t "$SESSION" tiled

echo "[5/5] Ready"
echo "Session: $SESSION"
echo "World  : $WORLD_NAME"
if [ "$ATTACH_TMUX" = "1" ]; then
    tmux attach-session -t "$SESSION"
else
    echo "ATTACH_TMUX=0, attach manually: tmux attach -t $SESSION"
fi
