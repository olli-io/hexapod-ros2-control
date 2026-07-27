#!/usr/bin/env bash
# Hexapod robot-image build + ship. Dispatched from `./hexa deploy <cmd>`.
#
# Two deploy targets:
#   (default)          the Raspberry Pi robot — an ARM64 container image.
#   --pico             the Pi Pico 2 W firmware — an RP2350 .uf2.
#
# Workstation-only commands (Pi target):
#   build              cross-build ARM64 image, save to .deploy/<sha>.tar.gz
#   push <host>        scp image + compose + launcher, ssh-load + start (energizes on launch)
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
SIM_COMPOSE_FILE="docker-compose.sim.yaml"
DEPLOY_DIR=".deploy"

usage() {
    cat <<EOF
Usage: ./hexa deploy [--pico] <command> [args...]

Raspberry Pi robot (ARM64 image):
  build                       Cross-build the ARM64 image and save to ${DEPLOY_DIR}/.
  push <host>                 scp + ssh-load the latest tarball to <host>, then start (energizes on launch).

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
    ssh "${host}" 'mkdir -p ~/hexa-robot ~/hexa-robot/log ~/hexa-robot/scripts ~/hexa-robot/systemd'

    # Ship the image + compose + env sample, plus the launcher (hexa +
    # scripts/robot.sh) so `hexa robot <cmd>` works on the Pi.
    echo ">> Shipping ${basename_tar} + compose + launcher to ${host}:~/hexa-robot/"
    scp \
        "${resolved}" \
        "${COMPOSE_FILE}" \
        ".env.robot.sample" \
        "hexa" \
        "${host}:~/hexa-robot/"
    scp "scripts/robot.sh" "${host}:~/hexa-robot/scripts/"
    # Boot-time systemd unit template. Shipped, never installed — the operator
    # opts in once with `./hexa robot install-service`, so a redeploy can't
    # silently change the Pi's systemd state.
    scp "systemd/hexa-robot.service" "${host}:~/hexa-robot/systemd/"
    # Buzzer: the player plus its four unit templates. Same deal — shipped, never
    # installed; `./hexa robot install-tune` is the operator's opt-in. Runs on
    # the Pi host rather than in the container, both so it chirps before Docker
    # is up and because the container cannot reach the PWM sysfs at all; the
    # spool pair is how the container's own beeps get out.
    scp "systemd/buzzer.sh" \
        "systemd/hexa-boot-tune.service" \
        "systemd/hexa-shutdown-tune.service" \
        "systemd/hexa-tune-spool.path" \
        "systemd/hexa-tune-spool.service" \
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
chmod +x systemd/buzzer.sh
docker compose -f "${COMPOSE_FILE}" up -d --no-build
EOF

    echo ">> Deployed and energized. The servo rail closes once teleop publishes; the"
    echo "   robot takes up the folded pose one leg at a time and stops there."
    echo "   Stand it:       gamepad Start (or publish /gait/initialize)."
    echo "   Status:         hexa robot -H ${host} status"
    echo "   Start on boot:  ssh -t ${host} 'cd ~/hexa-robot && ./hexa robot install-service'"
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
    build)      cmd_build "$@" ;;
    push)       cmd_push "$@" ;;
    -h|--help)  usage ;;
    *)
        echo "hexa deploy: unknown command '${sub}'" >&2
        usage >&2
        exit 1
        ;;
esac
