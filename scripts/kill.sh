#!/usr/bin/env bash
# Tear down the hexapod sim + pico containers (docker compose down).
# No-op if nothing is up.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

# Host's `input` group GID, needed for the compose interpolation. Falls back
# to 992 (matches scripts/sim.sh).
input_gid() {
    local gid
    gid="$(getent group input 2>/dev/null | cut -d: -f3 || true)"
    echo "${gid:-992}"
}

# `UID` is a readonly bash builtin, so pass UID/GID/INPUT_GID inline as env.
env UID="$(id -u)" GID="$(id -g)" INPUT_GID="$(input_gid)" \
    docker compose -f docker-compose.sim.yaml down --remove-orphans

echo "Removed the sim + pico containers."
