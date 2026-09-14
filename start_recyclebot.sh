#!/usr/bin/env bash
# One-click start for RecycleBot: container, workspace build, operator dashboard.
#
#   ./start_recyclebot.sh                      start everything, open the dashboard
#   ./start_recyclebot.sh --install-shortcut   put a "RecycleBot" icon on the desktop
#   ./start_recyclebot.sh --stop               stop the robot software and dashboard
#
# This does the README's every-run steps — export_env.sh, docker compose up,
# colcon build when sources changed — and then starts the dashboard, which takes
# over ros2 launch and the /launch_gate call. One-time setup (Docker, the
# real-time kernel, git lfs, calibration) is still a technician's job.
#
# Linux only: the dashboard relies on the dev container's host networking.

REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
CONTAINER="recyclebot-dev-1"
IMAGE="ros2_dev"
PORT="${RECYCLEBOT_DASHBOARD_PORT:-8080}"
URL="http://localhost:${PORT}"
BUILD_PACKAGES="grip_interface grip_command_package recycle_bot_moveit_config recycle_bot test_suite"
DASHBOARD_PROC="lib/recycle_bot/dashboard"   # pkill -f pattern for the node

step() { printf '\n\033[1;32m==>\033[0m \033[1m%s\033[0m\n' "$*"; }

fail() {
    printf '\n\033[1;31mProblem:\033[0m %s\n' "$1"
    [ -n "$2" ] && printf '%s\n' "$2"
    # Started from the desktop icon, the terminal closes when this script exits;
    # keep the message on screen until someone has read it.
    [ -t 0 ] && read -r -p $'\nPress Enter to close this window.' _
    exit 1
}

compose() {
    docker compose --env-file .env -f docker-compose.base.yml -f docker-compose.dev.yml "$@"
}

in_container() {
    docker exec "$CONTAINER" /ros_entrypoint.sh bash -c "$1"
}

# Prints the dashboard's state JSON if it answers, nothing otherwise. python3
# rather than curl: a stock Ubuntu desktop does not ship curl.
dashboard_state() {
    python3 - "$URL/api/state" 2>/dev/null <<'EOF'
import sys, urllib.request
print(urllib.request.urlopen(sys.argv[1], timeout=2).read().decode())
EOF
}

stop_dashboard() {
    # SIGTERM makes the dashboard stop any running robot launch before it exits
    # (the real robot takes up to ~30 s to shut down cleanly), so wait for it.
    docker exec "$CONTAINER" pkill -TERM -f "$DASHBOARD_PROC" 2>/dev/null || return 1
    for _ in $(seq 1 45); do
        docker exec "$CONTAINER" pgrep -f "$DASHBOARD_PROC" >/dev/null 2>&1 || return 0
        sleep 1
    done
    return 0
}

install_shortcut() {
    local app_dir="$HOME/.local/share/applications"
    local desktop_file="$app_dir/recyclebot.desktop"
    mkdir -p "$app_dir"
    cat > "$desktop_file" <<EOF
[Desktop Entry]
Type=Application
Name=RecycleBot
Comment=Start the RecycleBot sorting robot and open its dashboard
Exec=bash "$REPO_DIR/start_recyclebot.sh"
Icon=$REPO_DIR/packages/recycle_bot/web/icon.svg
Terminal=true
Categories=Utility;
EOF
    chmod +x "$desktop_file"
    echo "Added RecycleBot to the applications menu."

    local desktop_dir
    desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
    if [ -d "$desktop_dir" ]; then
        cp "$desktop_file" "$desktop_dir/recyclebot.desktop"
        chmod +x "$desktop_dir/recyclebot.desktop"
        # GNOME refuses to launch untrusted desktop files ("Allow Launching")
        gio set "$desktop_dir/recyclebot.desktop" metadata::trusted true 2>/dev/null || true
        echo "Added a RecycleBot icon to $desktop_dir."
        echo "If it shows a warning sign, right-click it and choose 'Allow Launching'."
    fi
}

cd "$REPO_DIR" || fail "Cannot open $REPO_DIR."

case "$1" in
    "") ;;
    --install-shortcut) install_shortcut; exit 0 ;;
    --stop)
        step "Stopping RecycleBot"
        if stop_dashboard; then echo "Stopped."; else echo "The dashboard was not running."; fi
        exit 0 ;;
    -h|--help) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option '$1'. Try --help." >&2; exit 2 ;;
esac

echo "RecycleBot is starting. This window shows the progress —"
echo "you can close it once the dashboard has opened in the browser."

# -----------------------------------------------------------------------------
step "1/5  Checking Docker"
command -v docker >/dev/null 2>&1 \
    || fail "Docker is not installed." "See 'Install Docker' in README.md, or call a technician."
docker info >/dev/null 2>&1 \
    || fail "Docker is not running, or this user is not allowed to use it." \
            "Try 'sudo systemctl start docker', or call a technician."
echo "OK"

# -----------------------------------------------------------------------------
step "2/5  Preparing settings and screen access"
# Every run, not once: the xhost grants it applies are lost at each logout.
# shellcheck source=export_env.sh
source ./export_env.sh

# -----------------------------------------------------------------------------
step "3/5  Starting the robot container"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "First start on this computer: building the software. This takes about 30 minutes."
    compose build || fail "Building the software failed." \
                          "Call a technician (README.md, 'Docker Build & Launch')."
fi
# Only `up` a container that is not running: `up` on a running one whose
# settings changed would recreate it, killing whatever is running inside.
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    compose up -d || fail "Could not start the container."
fi
for _ in $(seq 1 30); do
    docker exec "$CONTAINER" true 2>/dev/null && break
    sleep 1
done
docker exec "$CONTAINER" true 2>/dev/null || fail "The container did not come up."
echo "OK"

# -----------------------------------------------------------------------------
step "4/5  Checking the robot software is up to date"
# Rebuild only when something under src/ changed since the last successful
# build. Config, launch files and the YOLO model are installed by copy, so an
# edit to any of them does nothing until this runs. The build uses whatever
# layout the workspace already has — switching between symlinked and copied
# installs in place is a known colcon failure.
build_log="$(mktemp)"
in_container '
    cd ~/ros2_ws || exit 1
    stamp=install/.dashboard_build_stamp
    if [ -f install/setup.bash ] && [ -f "$stamp" ] && [ -z "$(find src -newer "$stamp" -type f \
            -not -path "*/__pycache__/*" -not -path "*/.pytest_cache/*" \
            -not -path "*/test_output/*" -not -name "*.pyc" -print -quit 2>/dev/null)" ]; then
        echo "Already up to date."
        exit 0
    fi
    sym=""
    [ -L install/recycle_bot/share/recycle_bot/package.xml ] && sym="--symlink-install"
    echo "Changes found — rebuilding (a minute or two)..."
    colcon build --packages-select '"$BUILD_PACKAGES"' $sym || exit 1
    touch "$stamp"
    echo "@@REBUILT@@"
' 2>&1 | tee "$build_log" | grep -v "@@REBUILT@@"
build_status=${PIPESTATUS[0]}
rebuilt=no
grep -q "@@REBUILT@@" "$build_log" && rebuilt=yes
rm -f "$build_log"
[ "$build_status" -eq 0 ] || fail "Building the robot software failed." \
    "Someone may have changed the code or config. Call a technician."

# -----------------------------------------------------------------------------
step "5/5  Starting the dashboard"
state="$(dashboard_state)"
if [ -n "$state" ] && [ "$rebuilt" = yes ]; then
    if grep -q '"process_running": false' <<<"$state"; then
        echo "Restarting the dashboard to pick up the update..."
        stop_dashboard
        state=""
    else
        echo "The robot is running, so the dashboard keeps its current version until the next start."
    fi
fi
if [ -z "$state" ]; then
    docker exec -d "$CONTAINER" /ros_entrypoint.sh bash -c \
        "mkdir -p ~/logs && exec ros2 run recycle_bot dashboard --ros-args -p port:=$PORT >> ~/logs/dashboard.log 2>&1"
    for _ in $(seq 1 30); do
        state="$(dashboard_state)"
        [ -n "$state" ] && break
        sleep 1
    done
    [ -n "$state" ] || fail "The dashboard did not start." \
        "Its log is logs/dashboard.log in the recyclebot folder."
fi
echo "OK"

echo
echo "RecycleBot dashboard: $URL"
if [ -n "$DISPLAY$WAYLAND_DISPLAY" ] && command -v xdg-open >/dev/null 2>&1; then
    # setsid: the browser must survive this terminal window closing
    setsid xdg-open "$URL" >/dev/null 2>&1 &
    sleep 2
fi
echo "You can close this window."
