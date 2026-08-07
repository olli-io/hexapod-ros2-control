#!/usr/bin/env bash
# Hexapod robot installer for the Raspberry Pi. Run it on the Pi:
#
#   curl -fsSL https://raw.githubusercontent.com/olli-io/hexapod-ros2-control/main/install.sh | bash
#   curl -fsSL .../install.sh | bash -s -- --tag release-1.0.0 --start
#
# What it does, in order: check every dependency and refuse early if one is
# missing; download the release's ARM64 image tarball plus the matching support
# files (compose, launcher, systemd templates, tuning, buzzer player); load the
# image; seed ~/hexa-robot/.env from the sample with this Pi's own GIDs and
# device names filled in. It does NOT start the stack unless asked (--start):
# bringing the container up energizes the servos, which is the operator's call.
#
# This is the standalone counterpart to `hexa deploy push` from a workstation —
# same ~/hexa-robot/ layout, same files, no ssh and no cross-build.
set -euo pipefail

REPO_DEFAULT="olli-io/hexapod-ros2-control"
IMAGE_REPO="hexa-robot"
INSTALL_DIR_DEFAULT="${HOME}/hexa-robot"
# Free disk: compressed tarball (~205 MB) + the loaded image + slack.
REQUIRED_DISK_MB=2048
# Installed RAM, not free — 2 GB is the recommended board, and a 2 GB Pi reports
# a little under that once the firmware has taken its reservation.
RECOMMENDED_RAM_MB=1800

REPO="${HEXA_REPO:-${REPO_DEFAULT}}"
TAG=""
ASSET=""
INSTALL_DIR="${INSTALL_DIR_DEFAULT}"
DO_START=0
CHECK_ONLY=0
KEEP_ARCHIVE=0

if [ -t 1 ]; then
    C_BOLD=$'\033[1m'; C_RED=$'\033[31m'; C_YELLOW=$'\033[33m'
    C_GREEN=$'\033[32m'; C_OFF=$'\033[0m'
else
    C_BOLD=""; C_RED=""; C_YELLOW=""; C_GREEN=""; C_OFF=""
fi

say()  { echo "${C_BOLD}>>${C_OFF} $*"; }
ok()   { echo "   ${C_GREEN}ok${C_OFF}    $*"; }
warn() { echo "   ${C_YELLOW}warn${C_OFF}  $*"; }
bad()  { echo "   ${C_RED}fail${C_OFF}  $*"; }
die()  { echo "${C_RED}install: $*${C_OFF}" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: install.sh [options]

Options:
  --tag <tag>        Release to install (default: the latest release).
  --asset <name>     Image tarball asset to use, when a release carries several.
  --dir <path>       Install directory (default: ${INSTALL_DIR_DEFAULT}).
  --repo <owner/name>
                     GitHub repository (default: ${REPO_DEFAULT}).
  --start            Bring the stack up when the install finishes. Off by
                     default: 'up' energizes the servos and the robot takes up
                     its folded pose under power.
  --keep-archive     Keep the downloaded image tarball (deleted after a
                     successful load otherwise — SD cards are small).
  --check-only       Run the dependency checks and stop.
  -h, --help         Show this help.

Environment:
  GITHUB_TOKEN       Used for the GitHub API and asset download; needed only
                     if the repository is private.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --tag)          TAG="${2:-}"; [ -n "${TAG}" ] || die "--tag needs a value"; shift 2 ;;
        --asset)        ASSET="${2:-}"; [ -n "${ASSET}" ] || die "--asset needs a value"; shift 2 ;;
        --dir)          INSTALL_DIR="${2:-}"; [ -n "${INSTALL_DIR}" ] || die "--dir needs a value"; shift 2 ;;
        --repo)         REPO="${2:-}"; [ -n "${REPO}" ] || die "--repo needs a value"; shift 2 ;;
        --start)        DO_START=1; shift ;;
        --keep-archive) KEEP_ARCHIVE=1; shift ;;
        --check-only)   CHECK_ONLY=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        *)              usage >&2; die "unknown option '$1'" ;;
    esac
done

TMP_DIR=""
cleanup() { [ -n "${TMP_DIR}" ] && rm -rf "${TMP_DIR}"; }
trap cleanup EXIT

# ---------------------------------------------------------------- dependencies

FAILED=0
PI_MODEL=""

check_deps() {
    say "Checking dependencies"

    case "$(uname -s)" in
        Linux) ok "Linux host" ;;
        *)     bad "not Linux — the robot image is a Linux ARM64 container"; FAILED=1 ;;
    esac

    # The image is built for linux/arm64 only, and a 32-bit userland cannot run
    # it even on 64-bit silicon: install Raspberry Pi OS Lite (64-bit).
    case "$(uname -m)" in
        aarch64|arm64) ok "64-bit ARM ($(uname -m))" ;;
        armv7l|armv6l) bad "32-bit userland ($(uname -m)) — reflash with Raspberry Pi OS Lite (64-bit)"; FAILED=1 ;;
        *)             bad "unsupported architecture $(uname -m) — the image is linux/arm64"; FAILED=1 ;;
    esac

    if [ -n "${PI_MODEL}" ]; then
        case "${PI_MODEL}" in
            *"Raspberry Pi 5"*|*"Raspberry Pi 4"*) ok "${PI_MODEL}" ;;
            *"Raspberry Pi"*) warn "${PI_MODEL} — only the Pi 4 and Pi 5 are tested" ;;
            *) warn "unrecognised board: ${PI_MODEL}" ;;
        esac
    else
        warn "not a Raspberry Pi (no device-tree model) — continuing anyway"
    fi

    local cmd
    for cmd in curl tar gzip; do
        if command -v "${cmd}" >/dev/null 2>&1; then
            ok "${cmd}"
        else
            bad "missing ${cmd} — sudo apt install -y ${cmd}"; FAILED=1
        fi
    done

    if ! command -v docker >/dev/null 2>&1; then
        bad "missing docker — curl -fsSL https://get.docker.com | sh"
        FAILED=1
    elif ! docker info >/dev/null 2>&1; then
        # Almost always the group, not the daemon: get.docker.com starts it.
        bad "the Docker daemon is not reachable as $(id -un)"
        echo "         sudo usermod -aG docker $(id -un)   # then log out and back in"
        echo "         sudo systemctl enable --now docker"
        FAILED=1
    else
        ok "docker $(docker version -f '{{.Server.Version}}' 2>/dev/null || echo '')"
        if docker compose version >/dev/null 2>&1; then
            ok "docker compose $(docker compose version --short 2>/dev/null || echo '')"
        else
            bad "missing the compose v2 plugin — sudo apt install -y docker-compose-plugin"
            FAILED=1
        fi
    fi

    # Space on the filesystem that will hold the tarball and the loaded image.
    local probe="${INSTALL_DIR}" avail_mb
    while [ ! -d "${probe}" ] && [ "${probe}" != "/" ]; do probe="$(dirname "${probe}")"; done
    avail_mb="$(df -Pk "${probe}" 2>/dev/null | awk 'NR == 2 { print int($4 / 1024) }' || true)"
    if [ -z "${avail_mb}" ]; then
        warn "could not measure free space on ${probe}"
    elif [ "${avail_mb}" -lt "${REQUIRED_DISK_MB}" ]; then
        bad "${avail_mb} MB free on ${probe} — need about ${REQUIRED_DISK_MB} MB"
        FAILED=1
    else
        ok "${avail_mb} MB free on ${probe}"
    fi

    local ram_mb
    ram_mb="$(awk '/^MemTotal:/ { print int($2 / 1024) }' /proc/meminfo 2>/dev/null || true)"
    if [ -n "${ram_mb}" ] && [ "${ram_mb}" -lt "${RECOMMENDED_RAM_MB}" ]; then
        warn "${ram_mb} MB RAM installed — 2 GB total is the recommended minimum"
    elif [ -n "${ram_mb}" ]; then
        ok "${ram_mb} MB RAM installed"
    fi

    if command -v systemctl >/dev/null 2>&1; then
        ok "systemd (needed by 'hexa robot install-service')"
    else
        warn "no systemctl — start-on-boot, the boot tunes and the network button stay unavailable"
    fi

    [ "$(id -u)" -eq 0 ] && warn "running as root — the install lands in ${INSTALL_DIR}"

    if [ "${FAILED}" -ne 0 ]; then
        echo
        die "dependency checks failed — fix the items above and re-run."
    fi
    say "Dependencies satisfied"
}

# ------------------------------------------------------------------- hardware

# Not fatal, any of it: the display, the buttons and the buzzer are optional,
# and the UART is a config.txt line away. Reported once, before the download.
check_hardware() {
    say "Checking wiring and overlays (informational)"

    local uart="${SERVO_DEVICE_GUESS}"
    if [ -e "${uart}" ]; then
        ok "servo UART at ${uart}"
    else
        warn "no ${uart} — the Servo 2040 link is off"
        case "${PI_MODEL}" in
            *"Raspberry Pi 5"*) echo "         add dtparam=uart0=on to /boot/firmware/config.txt and reboot" ;;
            *"Raspberry Pi 4"*) echo "         sudo raspi-config nonint do_serial_hw 0 && sudo raspi-config nonint do_serial_cons 1" ;;
            *)                  echo "         enable the header UART on GPIO14/15 (docs/robot-environment.md §3)" ;;
        esac
    fi

    if [ -e /dev/spidev0.0 ]; then
        ok "SPI at /dev/spidev0.0 (OLED face)"
    else
        warn "no /dev/spidev0.0 — no face. Add dtparam=spi=on if the OLED is fitted."
    fi

    if [ -n "${BUZZER_PWM_GUESS}" ]; then
        ok "PWM block at ${BUZZER_PWM_GUESS} (buzzer)"
    else
        warn "no PWM block — no buzzer. Add dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 if one is fitted."
    fi
}

# ------------------------------------------------------- host-specific values

group_gid() { getent group "$1" 2>/dev/null | cut -d: -f3 || true; }

detect_host_values() {
    if [ -r /proc/device-tree/model ]; then
        PI_MODEL="$(tr -d '\0' < /proc/device-tree/model)"
    fi

    INPUT_GID_GUESS="$(group_gid input)"
    SPI_GID_GUESS="$(group_gid spi)"
    GPIO_GID_GUESS="$(group_gid gpio)"

    # Per-model kernel name for the GPIO14/15 UART; fall back to whichever node
    # actually exists on an unrecognised board.
    case "${PI_MODEL}" in
        *"Raspberry Pi 5"*) SERVO_DEVICE_GUESS="/dev/ttyAMA0" ;;
        *"Raspberry Pi 4"*) SERVO_DEVICE_GUESS="/dev/ttyS0" ;;
        *)
            if   [ -e /dev/ttyAMA0 ]; then SERVO_DEVICE_GUESS="/dev/ttyAMA0"
            elif [ -e /dev/ttyS0 ];   then SERVO_DEVICE_GUESS="/dev/ttyS0"
            else                           SERVO_DEVICE_GUESS="/dev/ttyAMA0"
            fi
            ;;
    esac

    # Reached by platform address: the pwmchipN number is kernel probe order.
    BUZZER_PWM_GUESS=""
    local d
    for d in /sys/bus/platform/devices/*.pwm/pwm; do
        if [ -d "${d}" ]; then BUZZER_PWM_GUESS="${d}"; break; fi
    done
}

# ------------------------------------------------------------- release lookup

api_get() {
    local url="$1"
    if [ -n "${GITHUB_TOKEN:-}" ]; then
        curl -fsSL -H "Authorization: Bearer ${GITHUB_TOKEN}" \
             -H "Accept: application/vnd.github+json" "${url}"
    else
        curl -fsSL -H "Accept: application/vnd.github+json" "${url}"
    fi
}

json_field() { sed -n "s/.*\"$2\": *\"\([^\"]*\)\".*/\1/p" <<<"$1" | head -1 || true; }

# One line per asset: name <TAB> api url <TAB> browser url.
#
# jq is not on a Pi OS Lite image, so this is sed. Records are cut at the one
# field that reliably starts an asset object — its own API url,
# .../releases/assets/<id> — rather than at '{': every asset embeds an uploader
# object, so brace-splitting tears each asset in half and loses its name. The
# uploader's own "url" (api.github.com/users/...) cannot be mistaken for the
# record boundary, which is what makes the split safe.
list_assets() {
    printf '%s' "$1" | tr -d '\n' \
        | sed 's|"url": *"https://api.github.com/repos/[^"]*/releases/assets/|\n&|g' \
        | grep 'browser_download_url' \
        | while IFS= read -r rec; do
            printf '%s\t%s\t%s\n' \
                "$(json_field "${rec}" name)" \
                "$(sed -n 's|.*"\(https://api.github.com/repos/[^"]*/releases/assets/[0-9]*\)".*|\1|p' <<<"${rec}")" \
                "$(json_field "${rec}" browser_download_url)"
        done || true
}

resolve_release() {
    local url
    if [ -n "${TAG}" ]; then
        url="https://api.github.com/repos/${REPO}/releases/tags/${TAG}"
        say "Looking up release ${TAG} in ${REPO}"
    else
        url="https://api.github.com/repos/${REPO}/releases/latest"
        say "Looking up the latest release in ${REPO}"
    fi

    local json
    json="$(api_get "${url}")" || die "could not reach the GitHub API for ${REPO}. Private repo? Set GITHUB_TOKEN. No releases yet? Pass --tag."
    RELEASE_TAG="$(json_field "${json}" tag_name)"
    [ -n "${RELEASE_TAG}" ] || die "no release found (asked for ${TAG:-latest})"

    local assets
    assets="$(list_assets "${json}")"
    [ -n "${assets}" ] || die "release ${RELEASE_TAG} carries no assets — is the image tarball uploaded?"

    local matches
    if [ -n "${ASSET}" ]; then
        matches="$(awk -F'\t' -v n="${ASSET}" '$1 == n' <<<"${assets}")"
        [ -n "${matches}" ] || die "release ${RELEASE_TAG} has no asset named '${ASSET}'"
    else
        # Narrowest match first: an arm64-tagged image, then any hexa-robot
        # image, then any tarball at all.
        matches="$(awk -F'\t' '$1 ~ /^'"${IMAGE_REPO}"'.*(arm64|aarch64).*\.tar\.gz$/' <<<"${assets}")"
        [ -n "${matches}" ] || matches="$(awk -F'\t' '$1 ~ /^'"${IMAGE_REPO}"'.*\.tar\.gz$/' <<<"${assets}")"
        [ -n "${matches}" ] || matches="$(awk -F'\t' '$1 ~ /\.tar\.gz$/' <<<"${assets}")"
        [ -n "${matches}" ] || die "release ${RELEASE_TAG} has no .tar.gz asset"
    fi

    if [ "$(wc -l <<<"${matches}")" -gt 1 ]; then
        echo "   several image assets on ${RELEASE_TAG}:" >&2
        cut -f1 <<<"${matches}" | sed 's/^/     /' >&2
        die "pick one with --asset <name>"
    fi

    ASSET_NAME="$(cut -f1 <<<"${matches}")"
    ASSET_API_URL="$(cut -f2 <<<"${matches}")"
    ASSET_URL="$(cut -f3 <<<"${matches}")"
    ok "release ${RELEASE_TAG}, asset ${ASSET_NAME}"
}

# ----------------------------------------------------------------- downloads

# The token path uses the API asset endpoint, the only one that works on a
# private repo. "$1" carries -C - on the resuming attempt.
curl_asset() {
    # shellcheck disable=SC2086  # word splitting is the point
    if [ -n "${GITHUB_TOKEN:-}" ]; then
        curl -fL --retry 3 --retry-delay 2 --progress-bar $1 \
             -H "Authorization: Bearer ${GITHUB_TOKEN}" \
             -H "Accept: application/octet-stream" \
             -o "${IMAGE_TARBALL}" "${ASSET_API_URL}"
    else
        curl -fL --retry 3 --retry-delay 2 --progress-bar $1 \
             -o "${IMAGE_TARBALL}" "${ASSET_URL}"
    fi
}

download_image() {
    IMAGE_TARBALL="${INSTALL_DIR}/${ASSET_NAME}"
    if [ -f "${IMAGE_TARBALL}" ] && gzip -t "${IMAGE_TARBALL}" 2>/dev/null; then
        ok "${ASSET_NAME} already downloaded"
        return
    fi

    say "Downloading ${ASSET_NAME} (this is the big one)"
    # 200 MB over a Pi's wifi is worth resuming, so try -C - first. Everything
    # that can go wrong with it — a server without byte ranges, junk left by a
    # previous run — is cured by throwing the file away and starting over, so a
    # failed resume is a retry, not the end of the install.
    local fresh_needed=0
    if [ -f "${IMAGE_TARBALL}" ]; then
        curl_asset "-C -" || fresh_needed=1
    else
        curl_asset "" || fresh_needed=1
    fi
    if [ "${fresh_needed}" -eq 0 ] && ! gzip -t "${IMAGE_TARBALL}" 2>/dev/null; then
        fresh_needed=1
    fi

    if [ "${fresh_needed}" -eq 1 ]; then
        warn "the download did not come through intact — starting it over"
        rm -f "${IMAGE_TARBALL}"
        curl_asset "" || die "download failed"
        gzip -t "${IMAGE_TARBALL}" 2>/dev/null || {
            rm -f "${IMAGE_TARBALL}"
            die "the downloaded tarball is corrupt — check the release asset and re-run"
        }
    fi
    ok "$(du -h "${IMAGE_TARBALL}" | cut -f1) downloaded"
}

# The files that live beside the image on the Pi — compose, launcher, systemd
# templates, tuning overlay, buzzer player. They are repo content, not image
# content, so they come from the source archive of the same tag: that is what
# keeps them in step with the image they were built alongside.
SUPPORT_PATHS=(
    'docker-compose.robot.yaml'
    'docker-compose.buzzer.yaml'
    '.env.robot.sample'
    'hexa'
    'scripts/robot.sh'
    'systemd/*'
    'src/hexa_description/config/tuning.yaml'
    'src/hexa_buzzer/hexa_buzzer/__init__.py'
    'src/hexa_buzzer/hexa_buzzer/tunes.py'
    'src/hexa_buzzer/hexa_buzzer/catalog.py'
    'src/hexa_buzzer/hexa_buzzer/pwm.py'
    'src/hexa_buzzer/hexa_buzzer/player.py'
    'src/hexa_buzzer/config/tunes.yaml'
    'src/hexa_buzzer/config/buzzer.yaml'
)

fetch_support_files() {
    say "Fetching the support files for ${RELEASE_TAG}"
    TMP_DIR="$(mktemp -d)"
    local src="${TMP_DIR}/source.tar.gz" stage="${TMP_DIR}/src"

    if [ -n "${GITHUB_TOKEN:-}" ]; then
        curl -fsSL -H "Authorization: Bearer ${GITHUB_TOKEN}" \
             -o "${src}" "https://api.github.com/repos/${REPO}/tarball/${RELEASE_TAG}" \
            || die "could not download the source archive for ${RELEASE_TAG}"
    else
        curl -fsSL -o "${src}" \
             "https://codeload.github.com/${REPO}/tar.gz/refs/tags/${RELEASE_TAG}" \
            || die "could not download the source archive for ${RELEASE_TAG}"
    fi

    mkdir -p "${stage}"
    local patterns=()
    local p
    for p in "${SUPPORT_PATHS[@]}"; do patterns+=("*/${p}"); done
    tar -xzf "${src}" -C "${stage}" --strip-components=1 --wildcards "${patterns[@]}" \
        || die "the source archive is missing files this installer needs"

    mkdir -p "${INSTALL_DIR}/log" "${INSTALL_DIR}/scripts" "${INSTALL_DIR}/systemd" \
             "${INSTALL_DIR}/hexa_buzzer/config"

    cp "${stage}/docker-compose.robot.yaml" \
       "${stage}/docker-compose.buzzer.yaml" \
       "${stage}/.env.robot.sample" \
       "${stage}/hexa" "${INSTALL_DIR}/"
    cp "${stage}/scripts/robot.sh" "${INSTALL_DIR}/scripts/"
    cp "${stage}"/systemd/* "${INSTALL_DIR}/systemd/"
    cp "${stage}/src/hexa_description/config/tuning.yaml" "${INSTALL_DIR}/tuning.yaml.default"
    cp "${stage}"/src/hexa_buzzer/hexa_buzzer/*.py "${INSTALL_DIR}/hexa_buzzer/"
    cp "${stage}"/src/hexa_buzzer/config/*.yaml "${INSTALL_DIR}/hexa_buzzer/config/"

    chmod +x "${INSTALL_DIR}/hexa" "${INSTALL_DIR}/scripts/robot.sh" \
             "${INSTALL_DIR}/systemd/network-mode.sh"
    ok "support files in ${INSTALL_DIR}"
}

# ---------------------------------------------------------------- image load

load_image() {
    say "Loading the image into Docker (a minute or two on an SD card)"
    local loaded
    loaded="$(gunzip -c "${IMAGE_TARBALL}" | docker load)" || die "docker load failed"
    printf '%s\n' "${loaded}" | sed 's/^/   /'

    # compose runs hexa-robot:latest. A release tarball may only carry the sha
    # tag, so point latest at whatever came in.
    if ! docker image inspect "${IMAGE_REPO}:latest" >/dev/null 2>&1; then
        local first
        first="$(printf '%s\n' "${loaded}" | sed -n "s/^Loaded image: \(${IMAGE_REPO}:.*\)$/\1/p" | head -1 || true)"
        [ -n "${first}" ] || die "the tarball contained no ${IMAGE_REPO} image"
        docker tag "${first}" "${IMAGE_REPO}:latest"
        ok "tagged ${first} as ${IMAGE_REPO}:latest"
    fi

    if [ "${KEEP_ARCHIVE}" -eq 1 ]; then
        ok "kept ${IMAGE_TARBALL}"
    else
        rm -f "${IMAGE_TARBALL}"
        ok "removed the tarball (--keep-archive keeps it)"
    fi
}

# --------------------------------------------------------------------- config

# Fill this Pi's own values into a freshly seeded .env — the GIDs and device
# names §4/§6 of docs/robot-environment.md otherwise ask you to look up by hand.
# Only keys we actually resolved are touched; the sample's default stands for
# the rest.
seed_env() {
    local sample="${INSTALL_DIR}/.env.robot.sample" env="${INSTALL_DIR}/.env"

    if [ -f "${env}" ]; then
        say ".env exists — keeping it, adding only keys it is missing"
        # Same shape as `hexa deploy sync-config`: append the missing keys under
        # their comment block, never touch a value already set.
        local missing
        missing="$(awk '
            FNR == NR {
                if ($0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/) {
                    k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
                    have[k] = 1
                }
                next
            }
            /^[[:space:]]*$/ { n = 0; shown = 0; next }
            /^[[:space:]]*#/ { buf[++n] = $0; next }
            $0 ~ /^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/ {
                k = $0; sub(/[[:space:]]*=.*$/, "", k); gsub(/[[:space:]]/, "", k)
                if (!(k in have)) {
                    if (!shown) { for (i = 1; i <= n; i++) print buf[i]; shown = 1 }
                    print $0
                }
            }
        ' "${env}" "${sample}")"

        if [ -n "${missing}" ]; then
            cp "${env}" "${env}.bak"
            {
                printf '\n# --- added by install.sh (%s): new defaults from .env.robot.sample ---\n' "${RELEASE_TAG}"
                printf '%s\n' "${missing}"
            } >> "${env}"
            printf '%s\n' "${missing}" \
                | awk '/^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/ { print "     + " $0 }'
            ok "appended the above (previous saved as .env.bak)"
        else
            ok "already has every key in the sample"
        fi
        return
    fi

    say "Seeding .env from the sample, with this Pi's values"
    cp "${sample}" "${env}"

    set_env_key() {
        local key="$1" value="$2"
        [ -n "${value}" ] || return 0
        sed -i "s|^${key}=.*|${key}=${value}|" "${env}"
        ok "${key}=${value}"
    }
    set_env_key INPUT_GID    "${INPUT_GID_GUESS}"
    set_env_key SPI_GID      "${SPI_GID_GUESS}"
    set_env_key GPIO_GID     "${GPIO_GID_GUESS}"
    set_env_key SERVO_DEVICE "${SERVO_DEVICE_GUESS}"
    set_env_key BUZZER_PWM   "${BUZZER_PWM_GUESS}"
}

# The tuning overlay tracks the release, like a deploy: the compose bind-mount
# shadows the image's baked copy, so a stale file here would pin an old schema.
# An on-Pi edit is kept as .bak rather than discarded.
seed_tuning() {
    local t="${INSTALL_DIR}/tuning.yaml"
    if [ -f "${t}" ] && ! cmp -s "${t}" "${INSTALL_DIR}/tuning.yaml.default"; then
        cp "${t}" "${t}.bak"
        warn "your tuning.yaml differed from the release — saved as tuning.yaml.bak"
    fi
    cp "${INSTALL_DIR}/tuning.yaml.default" "${t}"
    ok "tuning.yaml refreshed from ${RELEASE_TAG}"
}

# ----------------------------------------------------------------------- main

main() {
    echo "${C_BOLD}Hexapod robot installer${C_OFF} — ${REPO}"
    echo

    detect_host_values
    check_deps
    echo
    check_hardware

    if [ "${CHECK_ONLY}" -eq 1 ]; then
        echo
        say "Checks only — nothing installed."
        exit 0
    fi

    echo
    mkdir -p "${INSTALL_DIR}"
    local previous=""
    [ -f "${INSTALL_DIR}/.hexa-release" ] && previous="$(cat "${INSTALL_DIR}/.hexa-release")"

    resolve_release
    download_image
    fetch_support_files
    load_image
    echo
    seed_env
    seed_tuning
    printf '%s\n' "${RELEASE_TAG}" > "${INSTALL_DIR}/.hexa-release"

    echo
    if [ -n "${previous}" ] && [ "${previous}" != "${RELEASE_TAG}" ]; then
        say "Upgraded ${previous} -> ${RELEASE_TAG} in ${INSTALL_DIR}"
    else
        say "Installed ${RELEASE_TAG} in ${INSTALL_DIR}"
    fi

    local running=""
    running="$(docker inspect -f '{{.State.Status}}' hexa-robot 2>/dev/null || true)"

    if [ "${DO_START}" -eq 1 ]; then
        echo
        say "Starting the stack (--start)"
        ( cd "${INSTALL_DIR}" && ./hexa robot restart ) \
            || die "start failed — check './hexa robot logs' in ${INSTALL_DIR}"
    else
        cat <<EOF

${C_BOLD}Next${C_OFF}
  1. Check ${INSTALL_DIR}/.env against this Pi (docs/robot-environment.md §6).
  2. cd ${INSTALL_DIR} && ./hexa robot up
EOF
        [ "${running}" = "running" ] && cat <<EOF
     The old container is still running the previous image — 'up' alone will not
     replace it. Use ./hexa robot restart.
EOF
        cat <<EOF
  3. Optional, each needs sudo on a TTY:
       ./hexa robot install-service   start the stack on power-on
       ./hexa robot install-tune      boot / shutdown buzzer tunes
       ./hexa robot install-network   hold the info button 3 s to flip to hotspot

  'up' energizes the servos: the robot takes up its folded pose one leg at a
  time and stops there. Standing it takes gamepad Start (or /gait/initialize).
EOF
    fi
}

main
