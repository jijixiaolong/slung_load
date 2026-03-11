#!/bin/bash
# ============================================================================
# Multi-UAV Slung Load — Universal Launch Script
# Usage:
#   ./launch.sh                    # default: 0.8m radius world
#   ./launch.sh standard           # same as above
#   ./launch.sh large_2m           # 2m inter-UAV spacing
#   ./launch.sh custom             # worlds/custom.sdf (if you generated one)
#   ./launch.sh my_world --dry-run # check paths without launching
# ============================================================================

set -e

# ── Config ───────────────────────────────────────────────────────────────────
WORLD_KEY="${1:-standard}"
DRY_RUN="${2:-}"
N_UAV=3

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GZ_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"          # Tools/simulation/gz/
PX4_DIR="$(cd "$GZ_DIR/../../../" && pwd)"       # PX4-Autopilot/

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_DIR/build/px4_sitl_default/etc"
PX4_ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
GZ_ENV="$PX4_ROOTFS/gz_env.sh"
GZ_WORLDS="$GZ_DIR/worlds"
MY_WORLDS="$SCRIPT_DIR/worlds"

# ── World file map ────────────────────────────────────────────────────────────
case "$WORLD_KEY" in
    standard|0.8m)  WORLD_NAME="multi_uav_slung_load_r0p80" ;;
    large_2m|2m)    WORLD_NAME="multi_uav_slung_load_r1p15" ;;
    *)              WORLD_NAME="$WORLD_KEY" ;;
esac

SDF_SRC="$MY_WORLDS/${WORLD_NAME}.sdf"
SDF_DST="$GZ_WORLDS/${WORLD_NAME}.sdf"

# ── Dry-run ────────────────────────────────────────────────────────────────────
if [ "$DRY_RUN" == "--dry-run" ]; then
    echo "=== Dry run ==="
    echo "PX4 binary : $PX4_BIN"
    echo "World file : $SDF_SRC → $SDF_DST"
    echo "gz_env.sh  : $GZ_ENV"
    [ -f "$PX4_BIN" ] && echo "PX4 binary  : OK" || echo "PX4 binary  : MISSING (run: make px4_sitl_default)"
    [ -f "$SDF_SRC" ] && echo "World SDF   : OK" || echo "World SDF   : MISSING (run: python3 gen_sdf.py)"
    [ -f "$GZ_ENV"  ] && echo "gz_env.sh   : OK" || echo "gz_env.sh   : MISSING"
    exit 0
fi

# ── Pre-checks ─────────────────────────────────────────────────────────────────
if [ ! -x "$PX4_BIN" ]; then
    echo "ERROR: PX4 binary not found: $PX4_BIN"
    echo "Build: cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

if [ ! -f "$GZ_ENV" ]; then
    echo "ERROR: gz_env.sh not found: $GZ_ENV"
    exit 1
fi

if [ ! -f "$SDF_SRC" ]; then
    echo "ERROR: World file not found: $SDF_SRC"
    echo ""
    echo "Generate it first:"
    echo "  cd $SCRIPT_DIR"
    case "$WORLD_KEY" in
        standard|0.8m) echo "  python3 gen_sdf.py --radius 0.8   --out worlds/${WORLD_NAME}.sdf" ;;
        large_2m|2m)   echo "  python3 gen_sdf.py --radius 1.1547 --out worlds/${WORLD_NAME}.sdf" ;;
        *)             echo "  python3 gen_sdf.py --radius <R>    --out worlds/${WORLD_NAME}.sdf" ;;
    esac
    exit 1
fi

# Install world file into Gazebo worlds directory
if [ ! -f "$SDF_DST" ] || [ "$SDF_SRC" -nt "$SDF_DST" ]; then
    echo ">>> Installing world file → $SDF_DST"
    cp "$SDF_SRC" "$SDF_DST"
fi

. "$GZ_ENV"

# ── Process management ────────────────────────────────────────────────────────
PX4_PIDS=()

kill_descendants() {
    local ppid="$1" child
    while read -r child; do
        [ -n "$child" ] || continue
        kill_descendants "$child"
        kill "$child" 2>/dev/null || true
    done < <(pgrep -P "$ppid" 2>/dev/null || true)
}

cleanup() {
    echo ""
    echo ">>> Stopping all instances..."
    for pid in "${PX4_PIDS[@]}"; do [ -n "$pid" ] && kill_descendants "$pid"; done
    for pid in "${PX4_PIDS[@]}"; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo ">>> Done."
}
trap cleanup EXIT

any_running() {
    for pid in "${PX4_PIDS[@]}"; do
        [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && return 0
    done
    return 1
}

start_instance() {
    local i="$1" wdir="$PX4_ROOTFS/$i" pid
    mkdir -p "$wdir"
    pushd "$wdir" >/dev/null
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_NAME="x500_$i" \
    PX4_GZ_WORLD="$WORLD_NAME" \
    "$PX4_BIN" -i "$i" -d "$PX4_ETC" > "/tmp/px4_${WORLD_NAME}_$i.log" 2>&1 &
    pid=$!
    popd >/dev/null
    PX4_PIDS+=("$pid")
    STARTED_PID="$pid"
}

# ── Launch ────────────────────────────────────────────────────────────────────
echo "========================================="
echo " Multi-UAV Slung Load Simulation"
echo " World  : $WORLD_NAME"
echo " UAVs   : $N_UAV × x500"
echo " Logs   : /tmp/px4_${WORLD_NAME}_*.log"
echo "========================================="

echo ">>> Starting PX4 instance 0 (+ Gazebo)..."
start_instance 0
echo "    PID=$STARTED_PID"

echo ">>> Waiting for Gazebo to start (~15s)..."
sleep 15

for i in $(seq 1 $((N_UAV-1))); do
    echo ">>> Starting PX4 instance $i..."
    start_instance "$i"
    echo "    PID=$STARTED_PID"
    sleep 3
done

echo ""
echo "========================================="
echo " All $N_UAV instances running!"
echo ""
echo " MicroXRCE-DDS agents (run in new terminals):"
for i in $(seq 0 $((N_UAV-1))); do
    port=$((8888 + i))
    echo "   MicroXRCEAgent udp4 -p $port &  # UAV $i"
done
echo ""
echo " Press Ctrl+C to stop all instances"
echo "========================================="

while true; do
    if ! wait -n "${PX4_PIDS[@]}" 2>/dev/null; then sleep 1; fi
    if ! any_running; then
        echo ">>> All PX4 processes exited."
        break
    fi
done
