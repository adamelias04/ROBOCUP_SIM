#!/bin/bash
# Start the whole RoboCup simulation: Isaac Sim (arena + robot) and the ROS 2
# side (robot description, lidar relay, RViz, steering GUI).
#
#   ./run.sh                 build on first run, then start everything
#   ./run.sh --build         rebuild the workspace first
#   ./run.sh --nav           also run Nav2 + slam_toolbox (goal: RViz 2D Goal Pose)
#   ./run.sh --nav --planner dwb    local planner: teb (default) | dwb | tr
#   ./run.sh --headless      any other option goes to the Isaac script
#                            (--headless, --x, --y, --yaw, --world FILE.usd)
#
# Both sides run in a clean environment with only /opt/ros/humble and this
# workspace, so a shell sourced for another workspace or set up for the real
# robot (RMW_IMPLEMENTATION=rmw_zenoh_cpp ...) cannot leak in.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PATH="${ISAACSIM_PATH:-$HOME/isaacsim}"
# Waiting for /clock covers Isaac's first start, which compiles shaders.
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
    # Isaac runs in its own process group (setsid); stop all of it.
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

# Stop everything as soon as either side ends (e.g. the Isaac window closed).
wait -n "$ISAAC_PID" "$LAUNCH_PID" || true
