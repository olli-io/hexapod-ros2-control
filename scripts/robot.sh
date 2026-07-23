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

# Boot-time systemd unit: the shipped template and where the rendered copy goes.
SERVICE_NAME="hexa-robot.service"
SERVICE_TEMPLATE="systemd/${SERVICE_NAME}"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

# Boot-jingle unit + player. A separate opt-in from the stack's unit above: the
# buzzer is optional hardware, and it runs on the host (not in the container),
# since the point is to chirp long before Docker exists.
TUNE_SERVICE_NAME="hexa-boot-tune.service"
TUNE_TEMPLATE="systemd/${TUNE_SERVICE_NAME}"
TUNE_SERVICE_PATH="/etc/systemd/system/${TUNE_SERVICE_NAME}"
TUNE_SCRIPT="systemd/boot-tune.sh"

# Name of the <ros2_control> block in the URDF. Must match the constant in
# hexa_bringup/launch/robot.launch.py.
HARDWARE_COMPONENT_NAME="HexaSystem"

usage() {
    cat <<EOF
Usage: ./hexa robot [-H user@host] <command> [args...]

Operate the local hexa-robot container. With -H/--host, re-dispatch the command
on a remote Pi over ssh (in ~/hexa-robot).

Commands:
  up                          compose up -d, then wait for the stack. The container
                              energizes itself on launch (HexaSystem active +
                              controllers), so this just brings it up. Drivable once
                              stood (gamepad Start closes the servo-rail relay).
  down                        Safe-stop: relay off + unload controllers, then compose down.
  restart                     down && up.
  boot                        Unattended 'up' for the systemd unit: wait for the
                              Docker daemon and the mapped device nodes, then up.
  install-service             Install + enable the hexa-robot systemd unit (needs
                              sudo), so 'boot' runs on power-on.
  uninstall-service           Disable + remove the systemd unit.
  install-tune                Install + enable the boot-jingle unit (needs sudo),
                              so the buzzer chirps on power-on. Optional hardware,
                              so it is a separate opt-in from install-service.
  uninstall-tune              Disable + remove the boot-jingle unit.
  play-tune                   Play the jingle now (buzzer bring-up, no reboot).
  status                      Container state + hardware-component state.
  logs [-f]                   docker compose logs.
  shell                       Interactive shell inside the container.

Teleop (gamepad + web) is part of the container's launch, so the robot is drivable
as soon as 'up' finishes. The container energizes on launch — HexaSystem goes active
and both controllers spawn — but energizing never powers the servo rail on its own:
the relay closes only once the robot stands (gamepad Start or /gait/initialize). A
'restart: unless-stopped' auto-restart therefore comes back energized but stationary,
so 'boot' is safe to run unattended.
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

# Block until controller_manager answers, so `up` reports ready only once the
# stack is live. `list_hardware_components` succeeds once it's up.
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
    compose up -d || die "compose up failed"
    # The container energizes itself on launch (robot.launch.py brings HexaSystem
    # active and spawns both controllers), so `up` just brings it up and waits for
    # the stack to report ready.
    wait_for_controller_manager
    echo ">> Robot is up and energized. Stand it (gamepad Start) to close the relay."
}

# Block until the Docker daemon answers. systemd's After=docker.service only
# guarantees the unit started, not that the socket is accepting connections.
wait_for_docker() {
    local _
    for _ in $(seq 1 60); do
        docker info >/dev/null 2>&1 && return 0
        sleep 1
    done
    die "Docker daemon did not become ready within 60s"
}

# Block until a /dev node exists. Compose maps each as a `devices:` entry and
# container creation fails outright if the path is missing, so wait rather than
# race: the UART is up early, but the SPI/GPIO nodes can lag the unit at boot.
wait_for_device() {
    local path="$1" label="$2" _
    for _ in $(seq 1 30); do
        [[ -e "${path}" ]] && return 0
        sleep 1
    done
    die "${label} ${path} did not appear within 30s"
}

# Unattended entry point for the systemd unit (see systemd/hexa-robot.service).
# Pre-flights the daemon and the device nodes compose maps, then runs the same
# `up` an operator would. Energizing spawns the controllers but leaves the servo
# rail open — the relay closes only once the robot stands — so this is safe to
# run with nobody watching.
cmd_boot() {
    [[ $# -eq 0 ]] || die "boot: unexpected argument '$1'"

    echo ">> Waiting for the Docker daemon..."
    wait_for_docker

    # Device paths come from the same .env compose interpolates, with the same
    # defaults. SPI / GPIO are only waited on when .env actually names them —
    # a robot without the OLED face fitted omits them.
    local servo_device="/dev/ttyAMA0" spi_device="" gpio_chip=""
    if [[ -f .env ]]; then
        # shellcheck disable=SC1091  # runtime file, not in the repo
        source <(grep -E '^(SERVO_DEVICE|SPI_DEVICE|GPIO_CHIP)=' .env || true)
        servo_device="${SERVO_DEVICE:-${servo_device}}"
        spi_device="${SPI_DEVICE:-}"
        gpio_chip="${GPIO_CHIP:-}"
    fi

    echo ">> Waiting for the Servo 2040 at ${servo_device}"
    wait_for_device "${servo_device}" "Servo 2040"
    [[ -n "${spi_device}" ]] && wait_for_device "${spi_device}" "SPI device"
    [[ -n "${gpio_chip}" ]] && wait_for_device "${gpio_chip}" "GPIO chip"

    cmd_up
}

# Render the shipped unit template for this user / install dir and enable it.
cmd_install_service() {
    [[ $# -eq 0 ]] || die "install-service: unexpected argument '$1'"
    [[ -f "${SERVICE_TEMPLATE}" ]] || die "missing ${SERVICE_TEMPLATE} — re-run 'hexa deploy push' to ship it"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    local rendered
    rendered="$(mktemp)"
    # shellcheck disable=SC2064  # expand ${rendered} now, at trap-set time
    trap "rm -f '${rendered}'" EXIT
    sed -e "s|@USER@|$(id -un)|g" -e "s|@HOME_DIR@|${REPO_ROOT}|g" \
        "${SERVICE_TEMPLATE}" > "${rendered}"

    echo ">> Installing ${SERVICE_PATH} (sudo)"
    sudo install -m 644 "${rendered}" "${SERVICE_PATH}" || die "install failed"
    sudo systemctl daemon-reload
    sudo systemctl enable "${SERVICE_NAME}"

    echo ">> Enabled. The stack will come up energized on every boot (rail stays open until stood)."
    echo "   Start it now:  sudo systemctl start ${SERVICE_NAME}"
    echo "   Watch it:      journalctl -u ${SERVICE_NAME} -f"
}

cmd_uninstall_service() {
    [[ $# -eq 0 ]] || die "uninstall-service: unexpected argument '$1'"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    echo ">> Disabling and removing ${SERVICE_PATH} (sudo)"
    sudo systemctl disable --now "${SERVICE_NAME}" || true
    sudo rm -f "${SERVICE_PATH}"
    sudo systemctl daemon-reload
    echo ">> Removed. 'hexa robot up' is the manual path again."
}

# Same render-and-enable dance as install-service, for the buzzer unit. Kept a
# separate opt-in because the buzzer is optional hardware.
cmd_install_tune() {
    [[ $# -eq 0 ]] || die "install-tune: unexpected argument '$1'"
    [[ -f "${TUNE_TEMPLATE}" ]] || die "missing ${TUNE_TEMPLATE} — re-run 'hexa deploy push' to ship it"
    [[ -f "${TUNE_SCRIPT}" ]] || die "missing ${TUNE_SCRIPT} — re-run 'hexa deploy push' to ship it"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    chmod +x "${TUNE_SCRIPT}"

    local rendered
    rendered="$(mktemp)"
    # shellcheck disable=SC2064  # expand ${rendered} now, at trap-set time
    trap "rm -f '${rendered}'" EXIT
    sed -e "s|@HOME_DIR@|${REPO_ROOT}|g" "${TUNE_TEMPLATE}" > "${rendered}"

    echo ">> Installing ${TUNE_SERVICE_PATH} (sudo)"
    sudo install -m 644 "${rendered}" "${TUNE_SERVICE_PATH}" || die "install failed"
    sudo systemctl daemon-reload
    sudo systemctl enable "${TUNE_SERVICE_NAME}"

    echo ">> Enabled. The jingle plays on every boot."
    echo "   Hear it now:   ./hexa robot play-tune"
    echo "   Watch it:      journalctl -u ${TUNE_SERVICE_NAME} -b"
}

cmd_uninstall_tune() {
    [[ $# -eq 0 ]] || die "uninstall-tune: unexpected argument '$1'"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    echo ">> Disabling and removing ${TUNE_SERVICE_PATH} (sudo)"
    sudo systemctl disable --now "${TUNE_SERVICE_NAME}" || true
    sudo rm -f "${TUNE_SERVICE_PATH}"
    sudo systemctl daemon-reload
    echo ">> Removed. The robot boots silent again."
}

# Play the jingle on demand — buzzer bring-up without a reboot. Needs root for
# the sysfs PWM export, same as the unit.
cmd_play_tune() {
    [[ $# -eq 0 ]] || die "play-tune: unexpected argument '$1' (tune it with the TUNE_* env vars instead)"
    [[ -f "${TUNE_SCRIPT}" ]] || die "missing ${TUNE_SCRIPT} — re-run 'hexa deploy push' to ship it"
    chmod +x "${TUNE_SCRIPT}" 2>/dev/null || true
    if [[ "$(id -u)" -eq 0 ]]; then
        "${TUNE_SCRIPT}"
    else
        # -E so a one-off `TUNE_MELODY=... ./hexa robot play-tune` survives sudo.
        sudo -E "${TUNE_SCRIPT}"
    fi
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
    boot)               cmd_boot "$@" ;;
    install-service)    cmd_install_service "$@" ;;
    uninstall-service)  cmd_uninstall_service "$@" ;;
    install-tune)       cmd_install_tune "$@" ;;
    uninstall-tune)     cmd_uninstall_tune "$@" ;;
    play-tune)          cmd_play_tune "$@" ;;
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
