#!/bin/bash
# Spusti celu simulaciu: Isaac Sim (arena + robot) a ROS 2 stranu (popis
# robota, relay lidaru, RViz, ovladanie cez rqt, volitelne Nav2).
#
#   ./run.sh                 pri prvom behu zbuilduje ws, potom spusti vsetko
#   ./run.sh --build         najprv prebuilduje ws
#   ./run.sh --nav           aj Nav2 + slam_toolbox (ciel cez 2D Goal Pose v RViz)
#   ./run.sh --nav --planner dwb    lokalny planner: teb (default) | dwb | tr
#   ./run.sh --headless      ostatne volby idu do Isaac skriptu
#                            (--headless, --x, --y, --yaw, --world SUBOR.usd)
#
# Obe strany bezia v cistom prostredi len s /opt/ros/humble a tymto ws, aby
# sa sem nedostal shell z ineho ws ani nastavenie pre realneho robota
# (RMW_IMPLEMENTATION=rmw_zenoh_cpp ...).
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
STARTUP_TIMEOUT=600

BUILD=0
NAV=false
PLANNER=teb
ISAAC_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --build) BUILD=1 ;;
        --nav) NAV=true ;;
        --planner) PLANNER="$2"; shift ;;
        *) ISAAC_ARGS+=("$1") ;;
    esac
    shift
done

clean_env() {
    env -i HOME="$HOME" USER="${USER:-}" PATH=/usr/bin:/bin TERM="${TERM:-xterm}" \
        DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" "$@"
}

ros() {
    clean_env RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash -c \
        "source /opt/ros/humble/setup.bash && source '$WS/install/setup.bash' && $1"
}

if [ ! -x "$ISAACSIM_PATH/python.sh" ]; then
    echo "Isaac Sim not found at $ISAACSIM_PATH (set ISAACSIM_PATH)" >&2
    exit 1
fi

if [ "$BUILD" = 1 ] || [ ! -f "$WS/install/setup.bash" ]; then
    echo "[run.sh] building $WS"
    clean_env bash -c "source /opt/ros/humble/setup.bash && cd '$WS' && colcon build --symlink-install"
fi

ISAAC_PID=""
LAUNCH_PID=""
cleanup() {
    trap - EXIT INT TERM
    [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
    [ -n "$ISAAC_PID" ] && kill -TERM -- "-$ISAAC_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run.sh] starting Isaac Sim"
clean_env setsid "$ISAACSIM_PATH/python.sh" "$WS/src/robocup_sim/isaac/robocup_isaac.py" \
    "${ISAAC_ARGS[@]}" &
ISAAC_PID=$!

echo "[run.sh] waiting for /clock from Isaac (first start can take minutes)"
for ((t = 0; t < STARTUP_TIMEOUT; t += 2)); do
    if ! kill -0 "$ISAAC_PID" 2>/dev/null; then
        echo "[run.sh] Isaac Sim exited before it was ready" >&2
        exit 1
    fi
    if ros "timeout 5 ros2 topic list 2>/dev/null" | grep -qx /clock; then
        break
    fi
    sleep 2
done
if ((t >= STARTUP_TIMEOUT)); then
    echo "[run.sh] no /clock after ${STARTUP_TIMEOUT}s" >&2
    exit 1
fi

echo "[run.sh] starting ROS 2 side"
ros "exec ros2 launch robocup_sim robocup.launch.py nav:=$NAV planner:=$PLANNER" &
LAUNCH_PID=$!

wait -n "$ISAAC_PID" "$LAUNCH_PID" || true
