#!/usr/bin/env bash
# Sim-container lifecycle via docker compose. Dispatched from `./hexa sim <cmd>`.
#
# The sim stack runs as the container's PID 1 (the composed sim_bringup launch),
# so its lifecycle is docker-native: `up -d` / `logs` / `down`. Builds and
# one-off commands run in ephemeral `compose run --rm` containers — there is no
# long-lived idle shell to attach to.
#
#   ./scripts/sim.sh up [--clean]          -> bring the sim stack up detached
#   ./scripts/sim.sh logs [-f]             -> show / stream its logs
#   ./scripts/sim.sh down                  -> stop and remove it
#   ./scripts/sim.sh build [args...]       -> colcon build in an ephemeral container
#   ./scripts/sim.sh shell                 -> interactive ROS2-sourced shell
#   ./scripts/sim.sh status                -> compose ps (+ node list when up)
#   ./scripts/sim.sh ros2 topic list       -> one-off command in the container
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

SERVICE="sim"
CONTAINER_NAME="hexa-sim"
PICO_SERVICE="pico"
PICO_CONTAINER_NAME="hexa-pico"

die() { echo "hexa sim: $*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: ./hexa sim <command> [args...]

Commands:
  up [--clean] [--pico] Bring the sim stack up detached (compose up -d). The
                        node implementation (C++ ports by default) is chosen by
                        the hexa_launch block of ros2_controllers.yaml, not a
                        flag; --clean rebuilds the image and the workspace.
                        --pico instead brings
                        up the firmware-in-sim brain (the Pi Pico firmware walks
                        the Gazebo hexapod); it builds hexa_pico_bridge first and
                        is mutually exclusive with the plain sim stack.
  down                  Stop and remove the sim container (compose down).
  refresh [args...]     Rebuild the workspace, then restart the running stack so it
                        picks up the changes. args pass through to colcon build
                        (e.g. --packages-select hexa_locomotion).
  restart               Restart the running stack WITHOUT rebuilding — applies
                        config-YAML edits (e.g. config/tuning.yaml), which are
                        already live via --symlink-install. Use 'refresh' for
                        source-code changes.
  logs [-f]             Show / stream the stack's logs.
  build [--clean] [args...]
                        colcon build in an ephemeral container (no stack needed).
                        --clean wipes build/ install/ log/ first for a full rebuild.
  shell                 Interactive ROS2-sourced shell (exec if up, else run --rm).
  face                  Terminal eye emulator: attach a live mirror of the face to
                        the running sim (the OLED is headless in sim). 'q' to detach;
                        closing it leaves the stack running.
  status                compose ps, plus 'ros2 node list' when the stack is up.
  <cmd...>              Run a one-off command in the container, e.g. 'ros2 topic list'.
EOF
}

# Host's `input` group GID, forwarded so the container user can read
# /dev/input/event* (needed by joy_node). Falls back to 992.
input_gid() {
    local gid
    gid="$(getent group input 2>/dev/null | cut -d: -f3 || true)"
    echo "${gid:-992}"
}

# `docker compose` with UID/GID/INPUT_GID pinned. `UID` is a readonly bash
# builtin, so it can't be exported — pass all three inline as env vars, which
# docker compose reads for the interpolations in docker-compose.sim.yaml.
compose() {
    env UID="$(id -u)" GID="$(id -g)" INPUT_GID="$(input_gid)" \
        docker compose -f docker-compose.sim.yaml "$@"
}

# TTY flags for interactive docker exec — allocate a TTY only when stdin is one,
# so piped / CI invocations still work.
tty_flags() {
    if [ -t 0 ]; then echo "-it"; else echo "-i"; fi
}

# Ephemeral one-shot container: `compose run --rm sim <cmd...>`. Disable the TTY
# when stdin isn't one so piped / CI invocations don't error.
compose_run() {
    local flags=(--rm)
    [ -t 0 ] || flags+=(-T)
    compose run "${flags[@]}" "${SERVICE}" "$@"
}

container_running() {
    [[ "$(docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || true)" == "running" ]]
}

sim_running()  { container_running "${CONTAINER_NAME}"; }
pico_running() { container_running "${PICO_CONTAINER_NAME}"; }

cmd_up() {
    local build_flags=() service="${SERVICE}" pico="" clean=""
    for a in "$@"; do
        case "$a" in
            --clean) build_flags=(--build); clean=1 ;;
            --pico)  pico=1; service="${PICO_SERVICE}" ;;
            *)       die "up: unknown flag '$a' (expected --clean or --pico)" ;;
        esac
    done
    # The image carries no built workspace (install/ is bind-mounted from the
    # host and built at runtime). On a fresh checkout, build once so the launch
    # can find the packages; later `up`s reuse the persisted install/. --clean
    # also forces a full workspace rebuild so source edits always take effect —
    # rebuilding the image (--build) recompiles nothing in the bind-mounted
    # install/, so C++ changes would otherwise be silently ignored. cmd_build
    # --clean wipes build/ install/ log/ first (see there).
    if [ -n "${clean}" ]; then
        echo "hexa sim: --clean — rebuilding the workspace from scratch..."
        cmd_build --clean
    elif [ ! -f install/setup.bash ]; then
        echo "hexa sim: no install/ yet — running an initial build..."
        cmd_build
    fi
    # --pico brings up the firmware-in-sim brain instead of the ROS2 node chain
    # (both drive the same Gazebo world, so they're mutually exclusive). Build
    # hexa_pico_bridge first so the launch can find it.
    if [ -n "${pico}" ]; then
        echo "hexa sim: building hexa_pico_bridge..."
        compose_run colcon build --symlink-install --packages-select hexa_pico_bridge
    fi
    compose up -d "${build_flags[@]}" "${service}"
    echo "hexa sim: up (detached, ${service}). Stream logs with 'hexa sim logs -f', stop with 'hexa sim down'."
}

cmd_down() { compose down; }

# Stream logs for whichever service is up (the pico brain if running, else sim).
cmd_logs() {
    local service="${SERVICE}"
    pico_running && service="${PICO_SERVICE}"
    compose logs "$@" "${service}"
}

# colcon build in an ephemeral container. A leading `--clean` wipes build/
# install/ log/ first, then does a full rebuild: --symlink-install leaves
# dangling symlinks behind when a source file is renamed or deleted, and
# colcon's install step then dies trying to copy the now-broken link. Remaining
# args pass through to colcon (e.g. --packages-select).
cmd_build() {
    if [ "${1:-}" = "--clean" ]; then
        shift
        echo "hexa sim: wiping build/ install/ log/..."
        rm -rf build install log
    fi
    compose_run colcon build --symlink-install "$@"
}

# Rebuild the (bind-mounted) workspace, then restart the running stack so the
# launch — PID 1 in the container — re-execs against the fresh install/. Build
# args (e.g. --packages-select) pass through to colcon for a fast single-package
# refresh.
cmd_refresh() {
    sim_running || die "refresh: sim stack isn't running — use 'hexa sim up' to start it."
    cmd_build "$@"
    echo "hexa sim: restarting the sim stack to pick up the rebuild..."
    compose restart "${SERVICE}"
    echo "hexa sim: refreshed. Stream logs with 'hexa sim logs -f'."
}

# Restart the running stack WITHOUT rebuilding. Config YAML is symlinked live
# into install/ (--symlink-install) and the repo is bind-mounted, so a bare
# restart re-execs PID 1 (the launch) and nodes re-read their config at startup
# — enough to apply a config/tuning.yaml value edit. For source-code changes use
# 'refresh' (build + restart) instead.
cmd_restart() {
    sim_running || die "restart: sim stack isn't running — use 'hexa sim up' to start it."
    echo "hexa sim: restarting the sim stack (no rebuild) to pick up config edits..."
    compose restart "${SERVICE}"
    echo "hexa sim: restarted. Stream logs with 'hexa sim logs -f'."
}

cmd_shell() {
    if sim_running; then
        # shellcheck disable=SC2046
        docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh bash
    else
        compose_run bash
    fi
}

# Terminal eye emulator. Execs the face_sim mirror into the running stack over a
# TTY, so it renders full-screen and 'q'/Ctrl-C closes only this sibling process
# — PID 1 (the sim launch) is untouched. use_sim_time so the policy clock tracks
# Gazebo; the policy params default to display.yaml's shipped values.
cmd_face() {
    sim_running || die "face: sim stack isn't running — use 'hexa sim up' first."
    # shellcheck disable=SC2046
    docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 run hexa_display face_sim --ros-args -p use_sim_time:=true
}

cmd_status() {
    compose ps
    local cname=""
    sim_running  && cname="${CONTAINER_NAME}"
    pico_running && cname="${PICO_CONTAINER_NAME}"
    if [ -n "${cname}" ]; then
        echo
        echo "Nodes:"
        docker exec "${cname}" /usr/local/bin/entrypoint.sh \
            ros2 node list 2>/dev/null || echo "  (ROS graph not responding yet)"
    fi
}

# One-off command: exec into the running stack if up, else an ephemeral run.
cmd_passthrough() {
    if sim_running; then
        # shellcheck disable=SC2046
        docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh "$@"
    elif pico_running; then
        # shellcheck disable=SC2046
        docker exec $(tty_flags) "${PICO_CONTAINER_NAME}" /usr/local/bin/entrypoint.sh "$@"
    else
        compose_run "$@"
    fi
}

# Allow the container to reach the host X server. Harmless when headless / CI.
xhost +local:docker >/dev/null 2>&1 || true

sub="${1:-}"
[ $# -gt 0 ] && shift || true

case "${sub}" in
    up)         cmd_up "$@" ;;
    down)       cmd_down "$@" ;;
    refresh)    cmd_refresh "$@" ;;
    restart)    cmd_restart "$@" ;;
    logs)       cmd_logs "$@" ;;
    build)      cmd_build "$@" ;;
    status)     cmd_status ;;
    face)       cmd_face ;;
    shell|"")   cmd_shell ;;
    -h|--help)  usage ;;
    *)          cmd_passthrough "${sub}" "$@" ;;
esac
