#!/usr/bin/env bash
# Sim-container lifecycle via docker compose. Dispatched from `./hexa sim <cmd>`.
#
# The sim stack runs as the container's PID 1 (the composed sim_bringup launch),
# so its lifecycle is docker-native: `up -d` / `logs` / `down`. Builds and
# one-off commands run in ephemeral `compose run --rm` containers — there is no
# long-lived idle shell to attach to.
#
#   ./scripts/sim.sh up [--cpp] [--clean]  -> bring the sim stack up detached
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

die() { echo "hexa sim: $*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: ./hexa sim <command> [args...]

Commands:
  up [--cpp] [--clean]  Bring the sim stack up detached (compose up -d). --cpp
                        runs the C++ ports (HEXA_CPP=1); --clean rebuilds the image.
  down                  Stop and remove the sim container (compose down).
  refresh [args...]     Rebuild the workspace, then restart the running stack so it
                        picks up the changes. args pass through to colcon build
                        (e.g. --packages-select hexa_gait).
  restart               Restart the running stack WITHOUT rebuilding — applies
                        config-YAML edits (e.g. config/tuning.yaml), which are
                        already live via --symlink-install. Use 'refresh' for
                        source-code changes.
  logs [-f]             Show / stream the stack's logs.
  build [args...]       colcon build in an ephemeral container (no stack needed).
  shell                 Interactive ROS2-sourced shell (exec if up, else run --rm).
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
# docker compose reads for the interpolations in docker-compose.sim.yaml. Any
# HEXA_CPP already exported by the caller rides along.
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

sim_running() {
    [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null || true)" == "running" ]]
}

cmd_up() {
    local build_flags=()
    for a in "$@"; do
        case "$a" in
            --clean) build_flags=(--build) ;;
            --cpp)   export HEXA_CPP=1 ;;
            *)       die "up: unknown flag '$a' (expected --cpp / --clean)" ;;
        esac
    done
    # The image carries no built workspace (install/ is bind-mounted from the
    # host and built at runtime). On a fresh checkout, build once so the launch
    # can find the packages; later `up`s reuse the persisted install/.
    if [ ! -f install/setup.bash ]; then
        echo "hexa sim: no install/ yet — running an initial build..."
        cmd_build
    fi
    compose up -d "${build_flags[@]}" "${SERVICE}"
    echo "hexa sim: up (detached). Stream logs with 'hexa sim logs -f', stop with 'hexa sim down'."
}

cmd_down() { compose down; }

cmd_logs() { compose logs "$@" "${SERVICE}"; }

cmd_build() { compose_run colcon build --symlink-install "$@"; }

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

cmd_status() {
    compose ps
    if sim_running; then
        echo
        echo "Nodes:"
        docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
            ros2 node list 2>/dev/null || echo "  (ROS graph not responding yet)"
    fi
}

# `hexa pico <sub>`: the firmware-in-sim service (Gazebo + joy + firmware bridge).
# Shares the sim image and this file's compose() helper.
cmd_pico() {
    local psub="${1:-up}"
    [ $# -gt 0 ] && shift || true
    case "${psub}" in
        up)
            local build_flags=()
            for a in "$@"; do
                case "$a" in
                    --clean) build_flags=(--build) ;;
                    *)       die "pico up: unknown flag '$a' (expected --clean)" ;;
                esac
            done
            echo "hexa pico: building hexa_pico_bridge..."
            compose_run colcon build --symlink-install --packages-select hexa_pico_bridge
            compose up -d "${build_flags[@]}" "${PICO_SERVICE}"
            echo "hexa pico: up (detached). Stream logs with 'hexa pico logs -f', stop with 'hexa pico down'."
            ;;
        down)       compose down ;;
        logs)       compose logs "$@" "${PICO_SERVICE}" ;;
        status)     compose ps ;;
        -h|--help)  echo "Usage: ./hexa pico <up [--clean] | down | logs [-f] | status>" ;;
        *)          die "pico: unknown command '${psub}' (expected up / down / logs / status)" ;;
    esac
}

# One-off command: exec into the running stack if up, else an ephemeral run.
cmd_passthrough() {
    if sim_running; then
        # shellcheck disable=SC2046
        docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh "$@"
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
    pico)       cmd_pico "$@" ;;
    status)     cmd_status ;;
    shell|"")   cmd_shell ;;
    -h|--help)  usage ;;
    *)          cmd_passthrough "${sub}" "$@" ;;
esac
