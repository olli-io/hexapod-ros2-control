#!/usr/bin/env bash
# Hexapod robot-container ops. Dispatched from `./hexa robot <cmd>`.

if [[ "${1:-}" == "-H" || "${1:-}" == "--host" ]]; then
    [[ -n "${2:-}" ]] || { echo "hexa robot: -H/--host needs a <user@host>" >&2; exit 1; }
    host="$2"
    shift 2
    ssh_flags=()
    [ -t 0 ] && ssh_flags=(-t)   # keep shell/teleop interactive; harmless for logs -f
    exec ssh "${ssh_flags[@]}" "${host}" "cd ~/hexa-robot && ./hexa robot $*"
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

CONTAINER_NAME="hexa-robot"
COMPOSE_FILE="docker-compose.robot.yaml"

# Name of the <ros2_control> block in the URDF. Must match the constant in
# hexa_bringup/launch/robot.launch.py.
HARDWARE_COMPONENT_NAME="HexaSystem"

usage() {
    cat <<EOF
Usage: ./hexa robot [-H user@host] <command> [args...]

Operate the local hexa-robot container. With -H/--host, re-dispatch the command
on a remote Pi over ssh (in ~/hexa-robot).

Commands:
  up                          compose up -d, then energize (relay on + spawn
                              controllers). Makes the robot drivable.
  down                        Safe-stop: relay off + unload controllers, then compose down.
  restart                     down && up.
  status                      Container state + hardware-component state.
  logs [-f]                   docker compose logs.
  shell                       Interactive shell inside the container.

Teleop (gamepad + web) is part of the container's launch, so the robot is drivable
as soon as 'up' finishes. The container always boots cold (relay open); 'up' is the
attended energize, so a 'restart: unless-stopped' auto-restart returns the robot to
a cold, safe state.
EOF
}

die() { echo "hexa robot: $*" >&2; exit 1; }

# Pick the host's `input` GID for compose. Matches scripts/sim.sh's logic.
input_gid() {
    local gid
    gid="$(getent group input 2>/dev/null | cut -d: -f3 || true)"
    echo "${gid:-994}"
}

# TTY flags for interactive docker exec — matches scripts/sim.sh:58-60.
tty_flags() {
    if [ -t 0 ]; then
        echo "-it"
    else
        echo "-i"
    fi
}

require_container_running() {
    local state
    state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
    [[ "${state}" == "running" ]] || die "container ${CONTAINER_NAME} is not running (state: ${state:-absent}). Run 'hexa robot up' first."
}

# `docker compose` invocation with the robot env / file pinned.
compose() {
    env \
        INPUT_GID="$(input_gid)" \
        docker compose -f "${COMPOSE_FILE}" "$@"
}

# Block until controller_manager answers, so energize() doesn't race a
# still-booting container. `list_hardware_components` succeeds once it's up.
wait_for_controller_manager() {
    echo ">> Waiting for controller_manager..."
    local _
    for _ in $(seq 1 60); do
        if docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
            ros2 control list_hardware_components >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    die "controller_manager did not come up within 60s"
}

cmd_up() {
    [[ $# -eq 0 ]] || die "up: unexpected argument '$1'"
    compose up -d
    wait_for_controller_manager
    energize
}

cmd_down() {
    # Safe-stop: drop the relay + unload the controllers first (if the container
    # is up), then remove the container.
    if [[ "$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null || true)" == "running" ]]; then
        deenergize
    fi
    compose down
}

cmd_restart() { cmd_down && cmd_up "$@"; }

cmd_logs() {
    if [[ "${1:-}" == "-f" ]]; then
        compose logs -f
    else
        compose logs "$@"
    fi
}

cmd_status() {
    docker ps --filter "name=^${CONTAINER_NAME}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    echo
    if docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null | grep -q running; then
        echo "Hardware components:"
        docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
            ros2 control list_hardware_components 2>/dev/null || \
            echo "  (controller_manager not responding yet)"
        echo
        echo "Active controllers:"
        docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
            ros2 control list_controllers 2>/dev/null || \
            echo "  (controller_manager not responding yet)"
    fi
}

cmd_shell() {
    require_container_running
    # shellcheck disable=SC2046
    docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh bash
}

# Relay ON + spawn controllers. Internal to `up` (no longer a public verb).
energize() {
    require_container_running
    echo ">> Activating ${HARDWARE_COMPONENT_NAME} (relay ON)"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 control set_hardware_component_state "${HARDWARE_COMPONENT_NAME}" active

    echo ">> Spawning joint_state_broadcaster"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 run controller_manager spawner joint_state_broadcaster

    echo ">> Spawning joint_group_position_controller"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 run controller_manager spawner joint_group_position_controller

    echo ">> Activated. Robot is now drivable."
}

# Unload controllers + relay OFF. Internal to `down` (no longer a public verb).
deenergize() {
    require_container_running
    echo ">> Unloading joint_group_position_controller"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 control unload_controller joint_group_position_controller || true

    echo ">> Unloading joint_state_broadcaster"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 control unload_controller joint_state_broadcaster || true

    echo ">> Deactivating ${HARDWARE_COMPONENT_NAME} (relay OFF)"
    docker exec "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 control set_hardware_component_state "${HARDWARE_COMPONENT_NAME}" inactive
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

sub="$1"
shift

case "${sub}" in
    up)         cmd_up "$@" ;;
    down)       cmd_down "$@" ;;
    restart)    cmd_restart "$@" ;;
    status)     cmd_status "$@" ;;
    logs)       cmd_logs "$@" ;;
    shell)      cmd_shell "$@" ;;
    -h|--help)  usage ;;
    *)
        echo "hexa robot: unknown command '${sub}'" >&2
        usage >&2
        exit 1
        ;;
esac
