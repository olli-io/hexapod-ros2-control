#!/usr/bin/env bash
# Hexapod prod-image lifecycle. Dispatched from `./hexa --prod <cmd>`.
#
# Workstation-only commands:
#   build              cross-build ARM64 image, save to .deploy/<sha>.tar.gz
#   deploy <host>      scp tarball + compose files, ssh-load + start cold
#
# Local (against the hexa-prod container, whether on the Pi or workstation):
#   start              docker compose up -d (cold; hardware in `inactive`)
#   stop               docker compose down
#   restart            stop && start (returns to cold state)
#   status             container + hardware-component state summary
#   logs [-f]          docker compose logs
#   shell              interactive ROS2-sourced shell in the container
#   engage             relay on: activate HexaSystem, spawn the controllers
#   disengage          inverse of engage (relay off)
#   teleop             re-launch teleop.launch.py inside the container
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

IMAGE_REPO="hexa-prod"
CONTAINER_NAME="hexa-prod"
COMPOSE_FILE="docker-compose.prod.yaml"
DEPLOY_DIR=".deploy"

# Name of the <ros2_control> block in the URDF. Must match the constant in
# hexa_bringup/launch/robot.launch.py.
HARDWARE_COMPONENT_NAME="HexaSystem"

usage() {
    cat <<EOF
Usage: ./hexa --prod <command> [args...]

Workstation:
  build                       Cross-build the ARM64 image and save to ${DEPLOY_DIR}/.
  deploy <host>               scp + ssh-load the latest tarball to <host>, then start cold.

Local container (Pi or workstation):
  start                       docker compose up -d (hardware boots cold/inactive).
  stop                        docker compose down.
  restart                     stop && start.
  status                      Container state + hardware-component state.
  logs [-f]                   docker compose logs.
  shell                       Interactive shell inside the container.
  engage                      Activate the hardware (relay on) and spawn controllers.
  disengage                   Unload controllers and deactivate the hardware (relay off).
  teleop                      Re-launch teleop inside the container.
EOF
}

die() { echo "hexa --prod: $*" >&2; exit 1; }

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

# Pick the host's `input` GID for compose. Matches scripts/dev.sh's logic.
input_gid() {
    local gid
    gid="$(getent group input 2>/dev/null | cut -d: -f3 || true)"
    echo "${gid:-994}"
}

# TTY flags for interactive docker exec — matches scripts/dev.sh:58-60.
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
    [[ "${state}" == "running" ]] || die "container ${CONTAINER_NAME} is not running (state: ${state:-absent}). Run 'hexa --prod start' first."
}

cmd_build() {
    require_cmd docker
    docker buildx version >/dev/null 2>&1 || die "docker buildx not available (install docker-buildx-plugin)"
    [[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]] \
        || die "aarch64 binfmt handler not registered — install qemu-user-static + qemu-user-static-binfmt (Arch) or equivalent, so cross-building linux/arm64 works."

    mkdir -p "${DEPLOY_DIR}"

    local sha
    sha="$(git rev-parse --short HEAD 2>/dev/null || date +%Y%m%d%H%M%S)"
    if ! git diff --quiet HEAD 2>/dev/null || ! git diff --quiet --cached 2>/dev/null; then
        sha="${sha}-dirty"
    fi

    local tag_sha="${IMAGE_REPO}:${sha}"
    local tag_latest="${IMAGE_REPO}:latest"

    echo ">> Building ${tag_sha} for linux/arm64"
    docker buildx build \
        --platform linux/arm64 \
        -f Dockerfile.prod \
        -t "${tag_sha}" \
        -t "${tag_latest}" \
        --output type=docker \
        .

    local tarball="${DEPLOY_DIR}/${IMAGE_REPO}_${sha}.tar.gz"
    echo ">> Saving ${tag_sha} to ${tarball}"
    docker save "${tag_sha}" "${tag_latest}" | gzip > "${tarball}"

    ln -sf "$(basename "${tarball}")" "${DEPLOY_DIR}/latest.tar.gz"

    local size
    size="$(du -h "${tarball}" | cut -f1)"
    echo ">> Done: ${tarball} (${size})"
}

cmd_deploy() {
    local host="${1:-}"
    [[ -n "${host}" ]] || die "usage: hexa --prod deploy <user@host>"

    require_cmd scp
    require_cmd ssh

    local tarball="${DEPLOY_DIR}/latest.tar.gz"
    [[ -e "${tarball}" ]] || die "no tarball at ${tarball}. Run 'hexa --prod build' first."

    # Resolve symlink so scp ships the actual file, not a dangling link.
    local resolved
    resolved="$(readlink -f "${tarball}")"
    local basename_tar
    basename_tar="$(basename "${resolved}")"

    echo ">> Ensuring ~/hexa-prod/ exists on ${host}"
    ssh "${host}" 'mkdir -p ~/hexa-prod ~/hexa-prod/log'

    echo ">> Shipping ${basename_tar} + compose files to ${host}:~/hexa-prod/"
    scp \
        "${resolved}" \
        "${COMPOSE_FILE}" \
        ".env.prod.sample" \
        "${host}:~/hexa-prod/"

    echo ">> Loading image and bringing service up (cold) on ${host}"
    # shellcheck disable=SC2087
    ssh "${host}" bash -s <<EOF
set -euo pipefail
cd ~/hexa-prod
gunzip -c "${basename_tar}" | docker load
# First-time provisioning: drop a .env from the sample if there isn't one.
[ -f .env ] || cp .env.prod.sample .env
docker compose -f "${COMPOSE_FILE}" up -d --no-build
EOF

    echo ">> Deployed. Service is up but the servo rail is cold."
    echo "   Engage with:   ssh ${host} '~/hexa-prod && hexa --prod engage'  (or run engage locally)"
}

# `docker compose` invocation with the prod env / file pinned.
compose() {
    env \
        INPUT_GID="$(input_gid)" \
        docker compose -f "${COMPOSE_FILE}" "$@"
}

cmd_start()   { compose up -d; }
cmd_stop()    { compose down; }
cmd_restart() { compose down && compose up -d; }

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

cmd_engage() {
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

    echo ">> Engaged. Robot is now drivable."
}

cmd_disengage() {
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

cmd_teleop() {
    require_container_running
    # shellcheck disable=SC2046
    docker exec $(tty_flags) "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh \
        ros2 launch hexa_teleop teleop.launch.py
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

sub="$1"
shift

case "${sub}" in
    build)      cmd_build "$@" ;;
    deploy)     cmd_deploy "$@" ;;
    start)      cmd_start "$@" ;;
    stop)       cmd_stop "$@" ;;
    restart)    cmd_restart "$@" ;;
    status)     cmd_status "$@" ;;
    logs)       cmd_logs "$@" ;;
    shell)      cmd_shell "$@" ;;
    engage)     cmd_engage "$@" ;;
    disengage)  cmd_disengage "$@" ;;
    teleop)     cmd_teleop "$@" ;;
    -h|--help)  usage ;;
    *)
        echo "hexa --prod: unknown command '${sub}'" >&2
        usage >&2
        exit 1
        ;;
esac
