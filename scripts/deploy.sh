#!/usr/bin/env bash
# Hexapod robot-image build + ship. Dispatched from `./hexa deploy <cmd>`.
#
# Workstation-only commands:
#   build              cross-build ARM64 image, save to .deploy/<sha>.tar.gz
#   push <host>        scp image + compose + launcher, ssh-load + start cold
#
# Once shipped, operate the running container with `hexa robot <cmd>` (see
# scripts/robot.sh) — locally on the Pi or remotely with `hexa robot -H <host>`.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

IMAGE_REPO="hexa-robot"
COMPOSE_FILE="docker-compose.robot.yaml"
DEPLOY_DIR=".deploy"

usage() {
    cat <<EOF
Usage: ./hexa deploy <command> [args...]

Workstation:
  build                       Cross-build the ARM64 image and save to ${DEPLOY_DIR}/.
  push <host>                 scp + ssh-load the latest tarball to <host>, then start cold.

Operate the shipped container with 'hexa robot <cmd>' (locally on the Pi, or
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
        -f Dockerfile.robot \
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
    ssh "${host}" 'mkdir -p ~/hexa-robot ~/hexa-robot/log ~/hexa-robot/scripts'

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

    echo ">> Loading image and bringing service up (cold) on ${host}"
    # shellcheck disable=SC2087
    ssh "${host}" bash -s <<EOF
set -euo pipefail
cd ~/hexa-robot
gunzip -c "${basename_tar}" | docker load
# First-time provisioning: drop a .env from the sample if there isn't one.
[ -f .env ] || cp .env.robot.sample .env
docker compose -f "${COMPOSE_FILE}" up -d --no-build
EOF

    echo ">> Deployed. Service is up but the servo rail is cold."
    echo "   Activate with:  ssh ${host} 'cd ~/hexa-robot && ./hexa robot activate'"
    echo "   or from here:   hexa robot -H ${host} activate"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
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
