#!/usr/bin/env bash
# Hexapod robot-image build + ship. Dispatched from `./hexa deploy <cmd>`.
#
# Two deploy targets:
#   (default)          the Raspberry Pi robot — an ARM64 container image.
#   --pico             the Pi Pico 2 W firmware — an RP2350 .uf2.
#
# Workstation-only commands (Pi target):
#   build [--fresh]    cross-build ARM64 image, save to .deploy/<sha>.tar.gz
#   push <host>        scp image + compose + launcher, ssh-load + start (energizes on launch)
#   sync-config <host> refresh the Pi's config from repo defaults — no image, no restart
#
# Pico target:
#   --pico [args...]   cross-build pi-pico-firmware/ inside the hexa-sim
#                      container (baked Pico SDK + ARM toolchain); the .uf2
#                      lands in pi-pico-firmware/build/. args -> cmake --build.
#
# Once the Pi image is shipped, operate the running container with `hexa robot
# <cmd>` (see scripts/robot.sh) — locally on the Pi or remotely with
# `hexa robot -H <host>`.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

IMAGE_REPO="hexa-robot"
COMPOSE_FILE="docker-compose.robot.yaml"
# Buzzer PWM overlay. Added only when the Pi has the tree — see robot.sh, which
# makes the same check for every other compose invocation.
BUZZER_COMPOSE_FILE="docker-compose.buzzer.yaml"
BUZZER_PWM_DEFAULT="/sys/bus/platform/devices/1f00098000.pwm/pwm"
SIM_COMPOSE_FILE="docker-compose.sim.yaml"
DEPLOY_DIR=".deploy"

usage() {
    cat <<EOF
Usage: ./hexa deploy [--pico] <command> [args...]

Raspberry Pi robot (ARM64 image):
  build [--fresh]             Cross-build the ARM64 image and save to ${DEPLOY_DIR}/.
                              Incremental (ccache + colcon build/ are cached across
                              builds); --fresh drops those caches and recompiles all.
  push <host>                 scp + ssh-load the latest tarball to <host>, then start (energizes on launch).
  sync-config <host> [--force]
                              Refresh config from repo defaults — no image, no restart.
                              Merges new keys into .env (existing values kept), overwrites
                              tuning.yaml, re-ships the host-side systemd payload.
                              --force overwrites .env outright. Backs up what it replaces.

Pi Pico 2 W firmware (RP2350 .uf2):
  --pico [args...]            Cross-build pi-pico-firmware/ in the hexa-sim container
                              (baked Pico SDK + ARM toolchain). The .uf2 lands in
                              pi-pico-firmware/build/; args pass to 'cmake --build'.

Operate the shipped Pi container with 'hexa robot <cmd>' (locally on the Pi, or
'hexa robot -H <host> <cmd>' from the workstation).
EOF
}

die() { echo "hexa deploy: $*" >&2; exit 1; }

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

# --fresh drops robot.Dockerfile's ccache / build/ mounts, for when stale
# incremental state is suspected. `--no-cache` does not clear cache mounts.
cmd_build() {
    local fresh=0 arg
    for arg in "$@"; do
        case "${arg}" in
            --fresh) fresh=1 ;;
            *)       die "unknown flag '${arg}' (usage: hexa deploy build [--fresh])" ;;
        esac
    done

    require_cmd docker
    docker buildx version >/dev/null 2>&1 || die "docker buildx not available (install docker-buildx-plugin)"
    [[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]] \
        || die "aarch64 binfmt handler not registered — install qemu-user-static + qemu-user-static-binfmt (Arch) or equivalent, so cross-building linux/arm64 works."

    if [[ "${fresh}" -eq 1 ]]; then
        echo ">> Dropping the build cache mounts (ccache + colcon build/)"
        docker builder prune --force --filter type=exec.cachemount >/dev/null
    fi

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
        -f robot.Dockerfile \
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

# Cross-build the RP2350 Pico firmware inside the hexa-sim container, whose image
# carries the baked Pico SDK + ARM toolchain + Bluepad32 + picotool (see
# sim.Dockerfile). The Pico 2 W is a deploy target like the Pi, so the build
# lives under `deploy`. An ephemeral `compose run --rm`; the workspace is bind-
# mounted, so hexa_pico.uf2 (+ .elf/.bin/.map) lands on the host under
# pi-pico-firmware/build/. UID/GID are pinned so the artifacts stay host-owned.
# Extra args forward to `cmake --build` (e.g. -v, --target clean). PICO_BOARD
# defaults to pico2_w (the firmware CMakeLists default).
cmd_pico() {
    require_cmd docker

    # `compose run` (ephemeral, one-shot) — NOT `compose up`, so this never
    # launches the ROS2 sim stack; it only spins a throwaway container to run
    # the cross-build. --build brings the hexa-sim image up to date first (so it
    # picks up the baked Pico toolchain from sim.Dockerfile) without a separate
    # image-build or sim-launch step; it's cached/fast when nothing changed.
    local flags=(--rm --build)
    [ -t 0 ] || flags+=(-T)

    echo ">> Building the Pi Pico firmware (RP2350) in a one-shot hexa-sim container"
    env UID="$(id -u)" GID="$(id -g)" \
        docker compose -f "${SIM_COMPOSE_FILE}" run "${flags[@]}" sim \
        bash -lc "set -e; cd pi-pico-firmware \
            && cmake -B build -DPICO_BOARD=pico2_w \
            && cmake --build build -j\"\$(nproc)\" $*"
    echo ">> Done: pi-pico-firmware/build/hexa_pico.uf2"
    echo "   Flash it by holding BOOTSEL and copying the .uf2 onto RPI-RP2,"
    echo "   or with 'picotool load -f pi-pico-firmware/build/hexa_pico.uf2'."
}

cmd_push() {
    local host="${1:-}"
    [[ -n "${host}" ]] || die "usage: hexa deploy push <user@host>"

    require_cmd scp
    require_cmd ssh

    local tarball="${DEPLOY_DIR}/latest.tar.gz"
    [[ -e "${tarball}" ]] || die "no tarball at ${tarball}. Run 'hexa deploy build' first."

    # Resolve symlink so scp ships the actual file, not a dangling link.
    local resolved
    resolved="$(readlink -f "${tarball}")"
    local basename_tar
    basename_tar="$(basename "${resolved}")"

    echo ">> Ensuring ~/hexa-robot/ exists on ${host}"
    ssh "${host}" 'mkdir -p ~/hexa-robot ~/hexa-robot/log ~/hexa-robot/scripts ~/hexa-robot/systemd ~/hexa-robot/hexa_buzzer ~/hexa-robot/hexa_buzzer/config'

    # Ship the image + compose + env sample, plus the launcher (hexa +
    # scripts/robot.sh) so `hexa robot <cmd>` works on the Pi.
    echo ">> Shipping ${basename_tar} + compose + launcher to ${host}:~/hexa-robot/"
    scp \
        "${resolved}" \
        "${COMPOSE_FILE}" \
        "${BUZZER_COMPOSE_FILE}" \
        ".env.robot.sample" \
        "hexa" \
        "${host}:~/hexa-robot/"
    scp "scripts/robot.sh" "${host}:~/hexa-robot/scripts/"
    # The buzzer player, run on the host by the boot and shutdown units. The
    # same files the container's hexa_buzzer node imports — shipped rather than
    # forked, so the note and tune tables cannot drift apart. Stdlib-only by
    # design (a pytest suite enforces it), so the host needs no ROS environment.
    scp "src/hexa_buzzer/hexa_buzzer/__init__.py" \
        "src/hexa_buzzer/hexa_buzzer/tunes.py" \
        "src/hexa_buzzer/hexa_buzzer/catalog.py" \
        "src/hexa_buzzer/hexa_buzzer/pwm.py" \
        "src/hexa_buzzer/hexa_buzzer/player.py" \
        "${host}:~/hexa-robot/hexa_buzzer/"
    # The tunes and the event -> tune map the player reads, beside it.
    scp "src/hexa_buzzer/config/tunes.yaml" \
        "src/hexa_buzzer/config/buzzer.yaml" \
        "${host}:~/hexa-robot/hexa_buzzer/config/"
    # Boot-time systemd unit template. Shipped, never installed — the operator
    # opts in once with `./hexa robot install-service`, so a redeploy can't
    # silently change the Pi's systemd state.
    scp "systemd/hexa-robot.service" "${host}:~/hexa-robot/systemd/"
    # Buzzer: the two unit templates for the tunes no container can play — the
    # boot chirp comes before Docker, the shutdown chirp after the container is
    # gone. Same deal as above: shipped, never installed, and `./hexa robot
    # install-tune` is the operator's opt-in. Everything in between is
    # hexa_buzzer's, played in the container, and needs no unit at all.
    scp "systemd/hexa-boot-tune.service" \
        "systemd/hexa-shutdown-tune.service" \
        "${host}:~/hexa-robot/systemd/"
    # Network-mode switcher, same deal again: shipped, never installed, and
    # `./hexa robot install-network` is the opt-in. It has to be on the host
    # because the container is unprivileged with no D-Bus socket, so it cannot
    # reach NetworkManager; the spool pair is how the info button's hold gets out.
    scp "systemd/network-mode.sh" \
        "systemd/hexa-network-spool.path" \
        "systemd/hexa-network-spool.service" \
        "systemd/hexa-network-report.service" \
        "${host}:~/hexa-robot/systemd/"
    # Runtime-tuning overlay. The compose bind-mounts ~/hexa-robot/tuning.yaml
    # over the image's copy so an on-Pi edit applies on a bare `hexa robot
    # restart` (no image rebuild). The repo stays the source of truth, so a
    # deploy always refreshes it — a seed-once overlay silently shadows the
    # image's fresh copy and pins whatever schema the Pi was first deployed
    # with. Any on-Pi edit is preserved as tuning.yaml.bak, not discarded.
    scp "src/hexa_description/config/tuning.yaml" \
        "${host}:~/hexa-robot/tuning.yaml.default"

    echo ">> Loading image and bringing service up on ${host} (energizes on launch)"
    # shellcheck disable=SC2087
    ssh "${host}" bash -s <<EOF
set -euo pipefail
cd ~/hexa-robot
gunzip -c "${basename_tar}" | docker load
# First-time provisioning: drop a .env from the sample if there isn't one.
# .env stays seed-once — it holds host-specific GIDs and device names that
# the repo cannot know.
[ -f .env ] || cp .env.robot.sample .env
# The tuning overlay always tracks the repo. Back up an on-Pi edit first so
# tuning done on the robot is recoverable rather than lost.
if [ -f tuning.yaml ] && ! cmp -s tuning.yaml tuning.yaml.default; then
    cp tuning.yaml tuning.yaml.bak
    echo ">> on-Pi tuning.yaml differed from the repo — saved as tuning.yaml.bak"
fi
cp tuning.yaml.default tuning.yaml
chmod +x systemd/network-mode.sh
# Superseded by hexa_buzzer — see the same line in sync-config.
rm -f systemd/buzzer.sh systemd/hexa-tune-spool.path systemd/hexa-tune-spool.service
# The buzzer's PWM mount, only when the Pi has the tree: a bind whose source is
# missing stops the container from starting at all. Same check robot.sh makes.
compose_args="-f ${COMPOSE_FILE}"
buzzer_pwm="\$(grep -E '^BUZZER_PWM=' .env 2>/dev/null | cut -d= -f2- || true)"
# An if, not a && short-circuit: under 'set -e' a false test would end the
# deploy on every robot that has no buzzer fitted.
if [ -d "\${buzzer_pwm:-${BUZZER_PWM_DEFAULT}}" ]; then
    compose_args="\${compose_args} -f ${BUZZER_COMPOSE_FILE}"
fi
# shellcheck disable=SC2086  # word splitting is the point
docker compose \${compose_args} up -d --no-build
EOF

    echo ">> Deployed and energized. The servo rail closes once teleop publishes; the"
    echo "   robot takes up the folded pose one leg at a time and stops there."
    echo "   Stand it:       gamepad Start (or publish /gait/initialize)."
    echo "   Status:         hexa robot -H ${host} status"
    echo "   Start on boot:  ssh -t ${host} 'cd ~/hexa-robot && ./hexa robot install-service'"
    echo "   Hotspot toggle: ssh -t ${host} 'cd ~/hexa-robot && ./hexa robot install-network'"
}

# Refresh the Pi's config from repo defaults — no image, no restart. Closes the
# gap seed-once .env leaves: a key added to .env.robot.sample after provisioning
# never reaches an existing robot. Merges the missing keys, keeps every value
# already set (those are host facts the repo cannot know); --force overwrites
# outright. Also re-ships tuning.yaml and the host-side systemd payload, which
# live outside the image. Units are shipped, never installed — systemd state
# stays the operator's opt-in. Image/compose/launcher are push's business:
# shipping them here would mean code with no matching image.
cmd_sync_config() {
    local host="" force=0 arg
    for arg in "$@"; do
        case "${arg}" in
            --force) force=1 ;;
            -*)      die "unknown flag '${arg}' (usage: hexa deploy sync-config <user@host> [--force])" ;;
            *)
                [[ -z "${host}" ]] || die "usage: hexa deploy sync-config <user@host> [--force]"
                host="${arg}"
                ;;
        esac
    done
    [[ -n "${host}" ]] || die "usage: hexa deploy sync-config <user@host> [--force]"

    require_cmd scp
    require_cmd ssh

    echo ">> Ensuring ~/hexa-robot/ exists on ${host}"
    ssh "${host}" 'mkdir -p ~/hexa-robot ~/hexa-robot/log ~/hexa-robot/scripts ~/hexa-robot/systemd ~/hexa-robot/hexa_buzzer ~/hexa-robot/hexa_buzzer/config'

    echo ">> Shipping config defaults to ${host}:~/hexa-robot/"
    scp ".env.robot.sample" "${host}:~/hexa-robot/"
    scp "src/hexa_description/config/tuning.yaml" "${host}:~/hexa-robot/tuning.yaml.default"
    scp "systemd/network-mode.sh" \
        "systemd/hexa-robot.service" \
        "systemd/hexa-boot-tune.service" \
        "systemd/hexa-shutdown-tune.service" \
        "systemd/hexa-network-spool.path" \
        "systemd/hexa-network-spool.service" \
        "systemd/hexa-network-report.service" \
        "${host}:~/hexa-robot/systemd/"
    # The host half of the buzzer, kept in step with the container's copy. See
    # the same block in cmd_push for why it is shipped rather than forked.
    scp "src/hexa_buzzer/hexa_buzzer/__init__.py" \
        "src/hexa_buzzer/hexa_buzzer/tunes.py" \
        "src/hexa_buzzer/hexa_buzzer/catalog.py" \
        "src/hexa_buzzer/hexa_buzzer/pwm.py" \
        "src/hexa_buzzer/hexa_buzzer/player.py" \
        "${host}:~/hexa-robot/hexa_buzzer/"
    scp "src/hexa_buzzer/config/tunes.yaml" \
        "src/hexa_buzzer/config/buzzer.yaml" \
        "${host}:~/hexa-robot/hexa_buzzer/config/"

    # Quoted heredoc — the merge awk's $0/$1 must reach the Pi unexpanded, so
    # the one parameter goes in as a positional arg.
    ssh "${host}" bash -s -- "${force}" <<'REMOTE'
set -euo pipefail
force="${1:-0}"
cd ~/hexa-robot

echo ">> .env"
if [ ! -f .env ]; then
    cp .env.robot.sample .env
    echo "   did not exist — seeded from .env.robot.sample"
elif [ "${force}" = "1" ]; then
    cp .env .env.bak
    cp .env.robot.sample .env
    echo "   overwritten from .env.robot.sample (previous saved as .env.bak)"
    echo "   host-specific values (INPUT_GID, SPI_GID, GPIO_GID, SERVO_DEVICE) are back"
    echo "   at the sample's guesses — check them against this Pi before restarting."
else
    # Pass 1: keys .env already sets. Pass 2: emit the ones it missed, each
    # under its comment block (once per block, so a group keeps its explanation).
    missing="$(awk '
        FNR == NR {
            if ($0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/) {
                k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
                have[k] = 1
            }
            next
        }
        /^[[:space:]]*$/  { n = 0; shown = 0; next }
        /^[[:space:]]*#/  { buf[++n] = $0; next }
        $0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/ {
            k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
            if (!(k in have)) {
                if (!shown) { for (i = 1; i <= n; i++) print buf[i]; shown = 1 }
                print $0
            }
        }
    ' .env .env.robot.sample)"

    if [ -n "${missing}" ]; then
        cp .env .env.bak
        {
            printf '\n# --- added by hexa deploy sync-config: new defaults from .env.robot.sample ---\n'
            printf '%s\n' "${missing}"
        } >> .env
        echo "   added (previous saved as .env.bak):"
        printf '%s\n' "${missing}" \
            | awk '/^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/ { print "     + " $0 }'
    else
        echo "   already has every key in .env.robot.sample — unchanged"
    fi

    # Keys only the Pi has: reported, never removed — a dropped key and a
    # deliberate local override look identical from here.
    stale="$(awk '
        FNR == NR {
            if ($0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/) {
                k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
                have[k] = 1
            }
            next
        }
        $0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/ {
            k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
            if (!(k in have)) print "     ? " k
        }
    ' .env.robot.sample .env)"
    if [ -n "${stale}" ]; then
        echo "   set here but absent from the sample (left alone):"
        printf '%s\n' "${stale}"
    fi
fi

echo ">> tuning.yaml"
if [ -f tuning.yaml ] && ! cmp -s tuning.yaml tuning.yaml.default; then
    cp tuning.yaml tuning.yaml.bak
    echo "   on-Pi copy differed from the repo — saved as tuning.yaml.bak"
fi
cp tuning.yaml.default tuning.yaml
echo "   refreshed from the repo"

chmod +x systemd/network-mode.sh
echo ">> systemd/ payload refreshed (shipped only — nothing installed, nothing reloaded)"
echo ">> hexa_buzzer/ player refreshed (run by the boot and shutdown units)"

# Superseded by hexa_buzzer, which plays the container's own beeps directly.
# Left-over copies would be a shell script nothing runs and a .path unit
# watching a spool nothing writes; './hexa robot install-tune' unhooks the
# units, this just clears the files a previous deploy left in place.
rm -f systemd/buzzer.sh systemd/hexa-tune-spool.path systemd/hexa-tune-spool.service

# The scripts are run by path, so a re-ship is live at once. Installed units are
# *rendered* copies (@HOME_DIR@ substituted), so they keep their install-time text.
for u in hexa-robot:install-service \
         hexa-boot-tune:install-tune \
         hexa-network-spool:install-network; do
    if [ -f "/etc/systemd/system/${u%%:*}.service" ]; then
        echo "   ${u%%:*}.service is installed — re-run './hexa robot ${u##*:}' to re-render it"
    fi
done
REMOTE

    echo ">> Config synced. Nothing was restarted."
    echo "   .env changes need a container recreate:  hexa robot -H ${host} restart"
    echo "   tuning.yaml is re-read on that same restart (no image rebuild)."
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

# --pico selects the Pico firmware target (leading flag, e.g. `hexa deploy
# --pico`); everything after it forwards to the firmware build.
if [[ "${1}" == "--pico" ]]; then
    shift
    cmd_pico "$@"
    exit 0
fi

sub="$1"
shift

case "${sub}" in
    build)        cmd_build "$@" ;;
    push)         cmd_push "$@" ;;
    sync-config)  cmd_sync_config "$@" ;;
    -h|--help)  usage ;;
    *)
        echo "hexa deploy: unknown command '${sub}'" >&2
        usage >&2
        exit 1
        ;;
esac
