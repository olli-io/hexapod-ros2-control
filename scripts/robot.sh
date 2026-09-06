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

# Hardware PWM for the buzzer, overlaid onto the compose file when the host
# actually has the tree. Optional hardware, and a bind mount whose source is
# missing stops the container from starting at all — so this is a real check,
# not a formality. Keep BUZZER_PWM in step with docker-compose.buzzer.yaml's
# default; .env may override it (a Pi 4's PWM block sits elsewhere).
BUZZER_COMPOSE_FILE="docker-compose.buzzer.yaml"
BUZZER_PWM_DEFAULT="/sys/bus/platform/devices/1f00098000.pwm/pwm"

# Boot-time systemd unit: the shipped template and where the rendered copy goes.
SERVICE_NAME="hexa-robot.service"
SERVICE_TEMPLATE="systemd/${SERVICE_NAME}"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

# Buzzer units. A separate opt-in from the stack's unit above: the buzzer is
# optional hardware, and installing the ROS stack should not silently start
# making noise.
#
# Only the two tunes that no container can play: the boot chirp lands long
# before Docker exists, and the shutdown chirp after the container is gone.
# Everything in between (up, fault, undervolt) is hexa_buzzer's, played in the
# container straight onto the PWM tree docker-compose.buzzer.yaml mounts. Both
# sides run the same Python — deploy ships it as ~/hexa-robot/hexa_buzzer/.
TUNE_MODULE="hexa_buzzer.player"
# Where `hexa_buzzer/` sits, deployed layout first, repo checkout second — so a
# dev machine can still audition a melody without a deploy. `python3 -m` finds
# it through PYTHONPATH; the units use WorkingDirectory instead, which is the
# same trick with fewer moving parts on a host that only ever has the first.
TUNE_PKG_DIRS=("." "src/hexa_buzzer")
#   hexa-boot-tune.service      the Pi is alive (boot)
#   hexa-shutdown-tune.service  power can be cut (shutdown)
TUNE_UNITS=(
    hexa-boot-tune.service
    hexa-shutdown-tune.service
)
# Both carry an [Install] section, unlike the network units' spool service.
TUNE_ENABLE_UNITS=("${TUNE_UNITS[@]}")
# Units from before hexa_buzzer, when the container asked for its beeps by
# writing log/buzzer and these relayed the request to a shell script. Removed on
# install and uninstall alike: a robot upgraded in place still has them enabled,
# watching a spool nothing writes, pointing at a buzzer.sh that is gone.
TUNE_LEGACY_UNITS=(
    hexa-tune-spool.path
    hexa-tune-spool.service
)

# Network-mode switcher + its units. Another separate opt-in: it rewrites
# NetworkManager profiles and can take the Pi off the network you are ssh'd in
# over, so nobody should get it just by deploying. Same shape as the buzzer —
# it has to run on the host because the container is unprivileged and has no
# D-Bus socket, so it cannot reach NetworkManager (see systemd/network-mode.sh).
NETWORK_SCRIPT="systemd/network-mode.sh"
#   hexa-network-spool.path/.service  relays the info button's hold
#   hexa-network-report.service       tells the container the mode at boot
NETWORK_UNITS=(
    hexa-network-spool.path
    hexa-network-spool.service
    hexa-network-report.service
)
# The subset with an [Install] section. hexa-network-spool.service is started by
# its .path and is deliberately not enablable on its own.
NETWORK_ENABLE_UNITS=(
    hexa-network-spool.path
    hexa-network-report.service
)

# mDNS. A third separate opt-in, and the most invasive of the three in one
# narrow way: it renames the Pi. That is the entire mechanism — avahi publishes
# <hostname>.local against whatever addresses the host has, on every interface,
# and re-announces by itself when they change. So a hostname is all it takes to
# be reachable by name in both network modes, and nothing here has to track an
# IP or hook NetworkManager. The cost is that `pi@raspberrypi` becomes
# `pi@hexa` in every ssh prompt and known_hosts, which is why nobody gets it by
# deploying.
MDNS_TEMPLATE="systemd/hexa-control.avahi-service"
MDNS_SERVICE_PATH="/etc/avahi/services/hexa-control.service"
# Beside network-mode.sh's previous-profile, and for the same reason: the thing
# this replaced has to be recoverable by uninstall.
MDNS_STATE_DIR="/var/lib/hexa-network"
MDNS_PREV_HOSTNAME="${MDNS_STATE_DIR}/previous-hostname"
MDNS_NAME="${HEXA_MDNS_NAME:-hexa}"

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
  install-tune                Install + enable the two buzzer units (needs sudo):
                              the boot and shutdown tunes, which no container is
                              running early or late enough to play. The stack's own
                              beeps (up, fault, undervolt) are hexa_buzzer's, in the
                              container, and need no unit. Optional hardware, so it
                              is a separate opt-in from install-service.
  uninstall-tune              Disable + remove the two buzzer units.
  play-tune [name]            Play a tune now, no reboot and no trip needed. Runs
                              the same player hexa_buzzer does, from the host, so it
                              works with the stack down. Takes an event from
                              buzzer.yaml (boot | up | shutdown | fault |
                              undervolt; the default is boot) or a tune from
                              tunes.yaml.
  install-network             Install + enable the network-mode units (needs
                              sudo), so holding the info button 3 s switches the
                              Pi between joining wifi and hosting the 'hexapod'
                              hotspot that serves the web teleop. Needs
                              NetworkManager (Pi OS Bookworm+). A separate
                              opt-in because it can take the Pi off the network
                              you are ssh'd in over.
  uninstall-network           Disable + remove the network-mode units.
  install-mdns                Install + enable mDNS (needs sudo), so the robot
                              answers to '${MDNS_NAME}.local' on any network it is on
                              and shows up in network browsers. Works in both
                              modes; needs no internet. A separate opt-in
                              because it RENAMES the Pi — avahi publishes
                              <hostname>.local, so the hostname is the
                              mechanism. Set HEXA_MDNS_NAME to pick another.
  uninstall-mdns              Remove the mDNS service and restore the previous
                              hostname.
  network-mode [mode]         Switch now, no button needed
                              (toggle | hotspot | station | status;
                              default status). Switching to the hotspot drops
                              any ssh session on wifi.
  status                      Container state + hardware-component state.
  logs [-f]                   docker compose logs.
  shell                       Interactive shell inside the container.

Teleop (gamepad + web) is part of the container's launch, so the robot is drivable
as soon as 'up' finishes. The container energizes on launch — HexaSystem goes active
and both controllers spawn. The servo rail closes once teleop is publishing and the
robot takes up its folded pose one leg at a time, then stops: standing takes a
gamepad Start (or /gait/initialize). A 'restart: unless-stopped' auto-restart
therefore comes back energized but stationary, so 'boot' is safe to run unattended.
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

# Where the host keeps the buzzer's PWM block, from the same .env compose
# interpolates, with the same default.
# The directory to put on PYTHONPATH so `python3 -m hexa_buzzer.player` resolves.
tune_pkg_dir() {
    local dir
    for dir in "${TUNE_PKG_DIRS[@]}"; do
        [[ -f "${dir}/hexa_buzzer/player.py" ]] && { echo "${dir}"; return 0; }
    done
    return 1
}

buzzer_pwm() {
    local dev="${BUZZER_PWM_DEFAULT}"
    if [[ -f .env ]]; then
        # shellcheck disable=SC1091  # runtime file, not in the repo
        source <(grep -E '^BUZZER_PWM=' .env || true)
        dev="${BUZZER_PWM:-${dev}}"
    fi
    echo "${dev}"
}

# `docker compose` invocation with the robot env / file pinned.
#
# The buzzer overlay is added only when the host has the PWM tree: its bind
# mount would otherwise stop the whole stack from starting on a robot with no
# buzzer fitted. Silent on the happy path — `up` says the noisy version once,
# where somebody is reading.
compose() {
    local files=(-f "${COMPOSE_FILE}")
    if [[ -d "$(buzzer_pwm)" ]]; then
        files+=(-f "${BUZZER_COMPOSE_FILE}")
    fi
    env \
        INPUT_GID="$(input_gid)" \
        docker compose "${files[@]}" "$@"
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
    local pwm_dev
    pwm_dev="$(buzzer_pwm)"
    if [[ -d "${pwm_dev}" ]]; then
        echo ">> Buzzer PWM at ${pwm_dev} -> /pwm"
    else
        echo ">> No PWM block at ${pwm_dev} — the buzzer stays silent."
        echo "   Fit one? Add dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4"
        echo "   to /boot/firmware/config.txt and reboot (docs/robot-environment.md §15)."
    fi
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
# `up` an operator would. Energizing spawns the controllers; the servo rail then
# closes and the robot settles into its folded pose leg by leg, but it never
# stands without a Start — so this is safe to run with nobody watching.
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

# Same render-and-enable dance as install-service, for the buzzer units. Kept a
# separate opt-in because the buzzer is optional hardware.
cmd_install_tune() {
    [[ $# -eq 0 ]] || die "install-tune: unexpected argument '$1'"
    tune_pkg_dir >/dev/null || die "missing hexa_buzzer/player.py — re-run 'hexa deploy push' to ship it"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"
    command -v python3 >/dev/null 2>&1 || die "python3 not found — the player needs it"

    local unit
    for unit in "${TUNE_UNITS[@]}"; do
        [[ -f "systemd/${unit}" ]] || die "missing systemd/${unit} — re-run 'hexa deploy push' to ship it"
    done

    remove_legacy_tune_units

    local rendered
    rendered="$(mktemp)"
    # shellcheck disable=SC2064  # expand ${rendered} now, at trap-set time
    trap "rm -f '${rendered}'" EXIT
    for unit in "${TUNE_UNITS[@]}"; do
        sed -e "s|@HOME_DIR@|${REPO_ROOT}|g" \
            -e "s|@PWM_DEV@|$(buzzer_pwm)|g" "systemd/${unit}" > "${rendered}"
        echo ">> Installing /etc/systemd/system/${unit} (sudo)"
        sudo install -m 644 "${rendered}" "/etc/systemd/system/${unit}" || die "install failed"
    done

    sudo systemctl daemon-reload
    sudo systemctl enable "${TUNE_ENABLE_UNITS[@]}"

    echo ">> Enabled. The buzzer now sounds on boot and on shutdown."
    echo "   The stack's own beeps (up, fault, undervolt) come from hexa_buzzer"
    echo "   in the container and need no unit — only the PWM mount, which"
    echo "   'hexa robot up' reports."
    echo "   Hear one now:  ./hexa robot play-tune [boot|up|shutdown|fault|undervolt]"
    echo "   Watch them:    journalctl -u hexa-boot-tune -u hexa-shutdown-tune -b"
}

# Clear out the pre-hexa_buzzer spool relay, wherever we find it. Quiet when
# there is nothing to do, which is every robot installed after the change.
remove_legacy_tune_units() {
    local unit found=0
    for unit in "${TUNE_LEGACY_UNITS[@]}"; do
        [[ -f "/etc/systemd/system/${unit}" ]] && found=1
    done
    [[ "${found}" -eq 1 ]] || return 0

    echo ">> Removing the old spool units, superseded by hexa_buzzer (sudo)"
    sudo systemctl disable --now "${TUNE_LEGACY_UNITS[@]}" 2>/dev/null || true
    for unit in "${TUNE_LEGACY_UNITS[@]}"; do
        sudo rm -f "/etc/systemd/system/${unit}"
    done
}

cmd_uninstall_tune() {
    [[ $# -eq 0 ]] || die "uninstall-tune: unexpected argument '$1'"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    echo ">> Disabling and removing the buzzer units (sudo)"
    sudo systemctl disable --now "${TUNE_ENABLE_UNITS[@]}" || true
    local unit
    for unit in "${TUNE_UNITS[@]}"; do
        sudo rm -f "/etc/systemd/system/${unit}"
    done
    remove_legacy_tune_units
    sudo systemctl daemon-reload
    echo ">> Removed. No more boot or shutdown chirp."
    echo "   The stack's own beeps are hexa_buzzer's and are unaffected — set"
    echo "   enabled: false in hexa_buzzer/config/buzzer.yaml to stop those."
}

# Same render-and-enable dance again, for the network-mode units. Kept separate
# from install-tune because this one changes how the Pi is reachable.
cmd_install_network() {
    [[ $# -eq 0 ]] || die "install-network: unexpected argument '$1'"
    [[ -f "${NETWORK_SCRIPT}" ]] || die "missing ${NETWORK_SCRIPT} — re-run 'hexa deploy push' to ship it"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"
    # Checked here rather than only at switch time so the failure lands where
    # somebody is watching, instead of on a 256x64 panel later.
    command -v nmcli >/dev/null 2>&1 || die \
        "nmcli not found — the switch needs NetworkManager (Pi OS Bookworm or newer)"

    local unit
    for unit in "${NETWORK_UNITS[@]}"; do
        [[ -f "systemd/${unit}" ]] || die "missing systemd/${unit} — re-run 'hexa deploy push' to ship it"
    done

    chmod +x "${NETWORK_SCRIPT}"

    local rendered
    rendered="$(mktemp)"
    # shellcheck disable=SC2064  # expand ${rendered} now, at trap-set time
    trap "rm -f '${rendered}'" EXIT
    for unit in "${NETWORK_UNITS[@]}"; do
        sed -e "s|@HOME_DIR@|${REPO_ROOT}|g" "systemd/${unit}" > "${rendered}"
        echo ">> Installing /etc/systemd/system/${unit} (sudo)"
        sudo install -m 644 "${rendered}" "/etc/systemd/system/${unit}" || die "install failed"
    done

    sudo systemctl daemon-reload
    sudo systemctl enable "${NETWORK_ENABLE_UNITS[@]}"
    # The spool watcher is what the button talks to, so arm it now rather than
    # leaving the gesture dead until the next reboot.
    sudo systemctl start hexa-network-spool.path
    # Wildcard DNS for the AP. Written once: NetworkManager only starts the
    # dnsmasq that reads it for a `shared` connection, so it is inert in station
    # mode and there is nothing to toggle.
    sudo "${NETWORK_SCRIPT}" --install-portal
    # Seed the state file so the container knows the mode without waiting for a
    # reboot or a button press.
    sudo "${NETWORK_SCRIPT}" --report --state "${REPO_ROOT}/log/network.state"

    echo ">> Enabled. Hold the info button 3 s to switch network mode."
    echo "   Hotspot:       SSID 'hexapod', password 'hexahexa', http://control.hexa/"
    echo "                  (a joining phone is offered it automatically)"
    echo "   Switch by hand: ./hexa robot network-mode [toggle|hotspot|station|status]"
    echo "   Watch it:      journalctl -u hexa-network-spool -u hexa-network-report -b"
    echo
    echo "   NOTE: wlan0 cannot be an access point and a client at once, so"
    echo "   switching to the hotspot WILL drop an ssh session on wifi. The Pi"
    echo "   always reboots back into station mode."
}

cmd_uninstall_network() {
    [[ $# -eq 0 ]] || die "uninstall-network: unexpected argument '$1'"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    echo ">> Disabling and removing the network-mode units (sudo)"
    sudo systemctl disable --now "${NETWORK_ENABLE_UNITS[@]}" || true
    local unit
    for unit in "${NETWORK_UNITS[@]}"; do
        sudo rm -f "/etc/systemd/system/${unit}"
    done
    sudo systemctl daemon-reload
    if [[ -f "${NETWORK_SCRIPT}" ]]; then
        # Drops the captive-DNS file and any port-80 redirect still standing.
        sudo "${NETWORK_SCRIPT}" --uninstall-portal || true
    fi
    # Leave the AP profile itself: deleting it would also throw away a hand-edited
    # SSID or channel. `nmcli connection delete hexapod-ap` if you want it gone.
    echo ">> Removed. The info button's hold no longer switches network mode."
}

# The port the service record advertises. webteleop.yaml is the source of truth,
# but `hexa deploy push` ships no src/ tree, so on the Pi that file is not there
# to read — hence a default, and an env override for anyone who has moved the
# port. Read where it exists (a full checkout), defaulted where it does not.
mdns_port() {
    local cfg="src/hexa_webteleop/config/webteleop.yaml"
    if [[ -n "${HEXA_MDNS_PORT:-}" ]]; then
        echo "${HEXA_MDNS_PORT}"
    elif [[ -f "${cfg}" ]] && grep -qE '^\s*port:' "${cfg}"; then
        sed -nE 's/^[[:space:]]*port:[[:space:]]*([0-9]+).*/\1/p' "${cfg}" | head -1
    else
        echo 8080
    fi
}

# mDNS, so a home network needs no address read off a 256x64 panel. The hotspot
# already solves this with a DHCP-advertised portal URL and unanswered probes
# (systemd/network-mode.sh); on somebody else's network there is no such hook,
# and this is the one mechanism that works without being the DHCP server.
cmd_install_mdns() {
    [[ $# -eq 0 ]] || die "install-mdns: unexpected argument '$1'"
    [[ -f "${MDNS_TEMPLATE}" ]] || die "missing ${MDNS_TEMPLATE} — re-run 'hexa deploy push' to ship it"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"
    # Checked here rather than discovered later on a phone that cannot resolve
    # the name. Pi OS ships it; a minimal image may not.
    command -v avahi-daemon >/dev/null 2>&1 || die \
        "avahi-daemon not found — install it first (apt install avahi-daemon)"
    command -v hostnamectl >/dev/null 2>&1 || die "hostnamectl not found"

    local port previous
    port="$(mdns_port)"
    previous="$(hostname)"

    if [[ "${previous}" != "${MDNS_NAME}" ]]; then
        echo ">> Renaming ${previous} -> ${MDNS_NAME} (sudo)"
        sudo install -d -m 755 "${MDNS_STATE_DIR}"
        # Only on the first rename: running install twice must not record
        # "hexa" as the name to go back to.
        [[ -f "${MDNS_PREV_HOSTNAME}" ]] || \
            echo "${previous}" | sudo tee "${MDNS_PREV_HOSTNAME}" >/dev/null
        sudo hostnamectl set-hostname "${MDNS_NAME}"
        # Without the matching /etc/hosts line every later sudo prints "unable
        # to resolve host" — cosmetic, but it lands on somebody mid-task.
        sudo sed -i -E "s/^(127\.0\.1\.1[[:space:]]+).*/\1${MDNS_NAME}/" /etc/hosts
        grep -qE "^127\.0\.1\.1[[:space:]]" /etc/hosts || \
            echo "127.0.1.1	${MDNS_NAME}" | sudo tee -a /etc/hosts >/dev/null
    fi

    local rendered
    rendered="$(mktemp)"
    # shellcheck disable=SC2064  # expand ${rendered} now, at trap-set time
    trap "rm -f '${rendered}'" EXIT
    sed -e "s|@PORT@|${port}|g" "${MDNS_TEMPLATE}" > "${rendered}"
    echo ">> Installing ${MDNS_SERVICE_PATH} (sudo)"
    sudo install -d -m 755 "$(dirname "${MDNS_SERVICE_PATH}")"
    sudo install -m 644 "${rendered}" "${MDNS_SERVICE_PATH}" || die "install failed"

    sudo systemctl enable --now avahi-daemon
    sudo systemctl reload-or-restart avahi-daemon

    echo ">> Enabled. The robot answers to ${MDNS_NAME}.local on any network it joins."
    echo "   Web teleop:    http://${MDNS_NAME}.local:${port}"
    echo "   Check it:      avahi-browse -rt _http._tcp   (from another machine)"
    echo
    echo "   To put the name on the panel instead of the address, set"
    echo "     mdns_name: \"${MDNS_NAME}.local\""
    echo "   in src/hexa_buttons/config/buttons.yaml. Left manual on purpose: the"
    echo "   container cannot tell whether this host actually runs avahi, and a"
    echo "   name on the panel that does not resolve strands whoever reads it."
    if [[ "${previous}" != "${MDNS_NAME}" ]]; then
        echo
        echo "   NOTE: the hostname changed. An open ssh session is fine, but the"
        echo "   next one is ${MDNS_NAME} — and the host key is now filed under a"
        echo "   new name, so expect a known_hosts prompt."
    fi
}

cmd_uninstall_mdns() {
    [[ $# -eq 0 ]] || die "uninstall-mdns: unexpected argument '$1'"
    command -v systemctl >/dev/null 2>&1 || die "systemctl not found — this host does not run systemd"

    echo ">> Removing ${MDNS_SERVICE_PATH} (sudo)"
    sudo rm -f "${MDNS_SERVICE_PATH}"

    if [[ -f "${MDNS_PREV_HOSTNAME}" ]]; then
        local previous
        previous="$(cat "${MDNS_PREV_HOSTNAME}")"
        echo ">> Restoring hostname ${previous}"
        sudo hostnamectl set-hostname "${previous}"
        sudo sed -i -E "s/^(127\.0\.1\.1[[:space:]]+).*/\1${previous}/" /etc/hosts
        sudo rm -f "${MDNS_PREV_HOSTNAME}"
    fi

    # Left running: avahi is a system service that predates us and may well be
    # what something else on this Pi is using. Only our record is ours to drop.
    sudo systemctl reload-or-restart avahi-daemon || true
    echo ">> Removed. The robot no longer advertises itself over mDNS."
}

# Switch by hand — network bring-up without a button fitted, and the only way to
# test the nmcli sequence before wiring it to the panel.
cmd_network_mode() {
    local mode="${1:-status}"
    [[ $# -le 1 ]] || die "network-mode: unexpected argument '$2'"
    [[ -f "${NETWORK_SCRIPT}" ]] || die "missing ${NETWORK_SCRIPT} — re-run 'hexa deploy push' to ship it"
    chmod +x "${NETWORK_SCRIPT}" 2>/dev/null || true
    local state="${REPO_ROOT}/log/network.state"
    if [[ "$(id -u)" -eq 0 ]]; then
        "${NETWORK_SCRIPT}" "${mode}" --state "${state}"
    else
        # -E so a one-off `HEXA_AP_SSID=... ./hexa robot network-mode` survives sudo.
        sudo -E "${NETWORK_SCRIPT}" "${mode}" --state "${state}"
    fi
}

# Play a tune on demand — buzzer bring-up without a reboot, and the only way to
# hear `up` / `fault` without provoking one. Runs the same player hexa_buzzer
# does, from the host, so it works with the stack down and needs no ROS
# environment. Needs root for the sysfs PWM export, same as the units.
#
# Safe to run while the stack is up: hexa_buzzer holds the channel only for the
# length of a tune, so the two never contend for longer than that.
cmd_play_tune() {
    local tune="${1:-boot}"
    [[ $# -le 1 ]] || die "play-tune: unexpected argument '$2'"
    command -v python3 >/dev/null 2>&1 || die "python3 not found — the player needs it"
    local args=(-m "${TUNE_MODULE}" "${tune}" --pwm-dev "$(buzzer_pwm)")
    local pkg
    pkg="$(tune_pkg_dir)" || die "missing hexa_buzzer/player.py — re-run 'hexa deploy push' to ship it"
    if [[ "$(id -u)" -eq 0 ]]; then
        PYTHONPATH="${pkg}" python3 "${args[@]}"
    else
        # sudo scrubs PYTHONPATH, so hand it over on the command line instead.
        sudo PYTHONPATH="${pkg}" python3 "${args[@]}"
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
    install-network)    cmd_install_network "$@" ;;
    uninstall-network)  cmd_uninstall_network "$@" ;;
    install-mdns)       cmd_install_mdns "$@" ;;
    uninstall-mdns)     cmd_uninstall_mdns "$@" ;;
    network-mode)       cmd_network_mode "$@" ;;
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
