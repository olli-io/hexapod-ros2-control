#!/usr/bin/env bash
# Drop into the hexapod ROS2 Jazzy dev container.
# Ensures a single long-lived container exists; multiple invocations attach
# to it instead of spawning new ones.
#   ./scripts/dev.sh                 -> interactive shell in the container
#   ./scripts/dev.sh ros2 topic list -> one-shot command in the container
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

CONTAINER_NAME="hexa-dev"

# Extract --test from the front of the arg list. When set, run colcon tests
# after the main command (which defaults to `pod build` in test mode).
run_tests=0
args=()
for arg in "$@"; do
    case "${arg}" in
        --test) run_tests=1 ;;
        *)      args+=("${arg}") ;;
    esac
done
set -- "${args[@]}"

# Allow the container to reach the host X server.
# Harmless if there is no X server (e.g. headless / CI).
xhost +local:docker >/dev/null 2>&1 || true

# Host's `input` group GID, forwarded so the container user can read
# /dev/input/event* (needed by joy_node when a controller is attached).
# Falls back to 992 if the host has no `input` group.
INPUT_GID="$(getent group input | cut -d: -f3)"
INPUT_GID="${INPUT_GID:-992}"

# Inspect the container once. Empty output = container doesn't exist.
state="$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}" 2>/dev/null || true)"

case "${state}" in
    running)
        ;;
    "")
        # No container yet — rebuild the image, then create and start it
        # detached. Always rebuilding on a fresh start means Dockerfile
        # edits take effect after `hexa kill && hexa --dev`,
        # without a separate rebuild step.
        # `UID` is a readonly builtin in bash, so we can't `export` it; pass
        # the values inline and docker compose reads them as env vars.
        env UID="$(id -u)" GID="$(id -g)" INPUT_GID="${INPUT_GID}" \
            docker compose up -d --build dev
        ;;
    *)
        # Exists but stopped (exited, created, paused, ...). Restart it.
        docker start "${CONTAINER_NAME}" >/dev/null
        ;;
esac

# Allocate a TTY only when stdin is one, so piped/CI invocations still work.
exec_flags=(-i)
[ -t 0 ] && exec_flags=(-it)

if [[ ${run_tests} -eq 1 ]]; then
    # In test mode the main command defaults to `pod build` so a bare
    # `dev.sh --test` means "build then test".
    docker exec "${exec_flags[@]}" "${CONTAINER_NAME}" \
        /usr/local/bin/entrypoint.sh "${@:-pod build}"
    exec docker exec "${exec_flags[@]}" "${CONTAINER_NAME}" \
        /usr/local/bin/entrypoint.sh bash -c \
        "colcon test --event-handlers console_direct+ && colcon test-result --verbose"
fi

exec docker exec "${exec_flags[@]}" "${CONTAINER_NAME}" /usr/local/bin/entrypoint.sh "${@:-bash}"
