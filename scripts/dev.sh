#!/usr/bin/env bash
# Drop into the hexapod ROS2 Jazzy dev container.
# Builds the image on first run; subsequent runs are fast.
# Pass any command after the script name; defaults to `bash`.
#   ./scripts/dev.sh                 -> interactive shell
#   ./scripts/dev.sh ros2 topic list -> one-shot command
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

# Allow the container to reach the host X server.
# Harmless if there is no X server (e.g. headless / CI).
xhost +local:docker >/dev/null 2>&1 || true

# Host's `input` group GID, forwarded so the container user can read
# /dev/input/event* (needed by joy_node when a controller is attached).
# Falls back to 992 if the host has no `input` group.
INPUT_GID="$(getent group input | cut -d: -f3)"
INPUT_GID="${INPUT_GID:-992}"

# `UID` is a readonly builtin in bash, so we can't `export` it.
# Pass the values inline; docker compose reads them as env vars.
exec env UID="$(id -u)" GID="$(id -g)" INPUT_GID="${INPUT_GID}" \
    docker compose run --rm --service-ports dev "${@:-bash}"
